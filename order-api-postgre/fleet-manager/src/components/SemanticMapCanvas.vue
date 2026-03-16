<template>
  <div
    ref="stageRef"
    class="semantic-map-stage"
    :class="{ 'semantic-map-stage-editing': editable }"
  >
    <canvas
      ref="canvasRef"
      class="semantic-map-canvas"
      @click="handleCanvasClick"
      @pointerdown="handlePointerDown"
      @pointermove="handlePointerMove"
      @pointerup="handlePointerUp"
      @pointerleave="handlePointerUp"
      @pointercancel="handlePointerUp"
      @wheel.prevent="handleWheel"
    />

    <div v-if="viewportReady" class="map-stage-hud">
      <span>{{ Math.round(viewport.scale * 100) }}%</span>
      <span>{{ fitLabel }}</span>
      <span v-if="editable">Edit</span>
    </div>
    <div v-if="loading" class="map-stage-banner">Loading map raster...</div>
    <div v-else-if="error" class="map-stage-banner map-stage-banner-error">{{ error }}</div>
  </div>
</template>

<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, reactive, ref, watch, watchEffect } from 'vue'
import { fetchWithAuth } from '../api'

const props = defineProps({
  bundle: {
    type: Object,
    default: null
  },
  robots: {
    type: Array,
    default: () => []
  },
  visibleLayers: {
    type: Object,
    default: () => ({})
  },
  selectedKey: {
    type: String,
    default: ''
  },
  interactive: {
    type: Boolean,
    default: true
  },
  editable: {
    type: Boolean,
    default: false
  },
  draggableTypes: {
    type: Array,
    default: () => ['anchor', 'rack', 'slot']
  },
  rasterOverride: {
    type: Object,
    default: null
  },
  autoFitMode: {
    type: String,
    default: 'content'
  }
})

const emit = defineEmits(['select', 'move-entity'])

const stageRef = ref(null)
const canvasRef = ref(null)
const raster = ref(null)
const loading = ref(false)
const error = ref('')
const stageSize = reactive({ width: 0, height: 0 })
const viewport = reactive({ scale: 1, centerX: 0, centerY: 0, mode: 'content' })
const viewportReady = ref(false)

let resizeObserver = null
let lastFitKey = ''
let suppressClick = false
let dragState = null

const activeRaster = computed(() => props.rasterOverride || raster.value)
const activeMap = computed(() => props.bundle?.map || null)
const mapWidth = computed(() => Number(activeMap.value?.width_px || activeRaster.value?.width || 0))
const mapHeight = computed(() => Number(activeMap.value?.height_px || activeRaster.value?.height || 0))
const fitLabel = computed(() => (viewport.mode === 'full' ? 'Full Map' : 'Fit Objects'))

watch(
  () => props.bundle?.map?.image_url,
  async (imageUrl) => {
    if (props.rasterOverride) {
      loading.value = false
      error.value = ''
      raster.value = null
      return
    }

    if (!imageUrl) {
      raster.value = null
      return
    }

    loading.value = true
    error.value = ''
    try {
      const response = await fetchWithAuth(imageUrl)
      if (!response.ok) {
        throw new Error(`Failed to load base map (${response.status})`)
      }
      const buffer = await response.arrayBuffer()
      raster.value = parsePgm(buffer)
    } catch (err) {
      raster.value = null
      error.value = err.message || 'Unable to load base map image'
    } finally {
      loading.value = false
    }
  },
  { immediate: true }
)

watch(
  () => `${activeMap.value?.name || ''}:${mapWidth.value}:${mapHeight.value}:${props.autoFitMode}`,
  async (fitKey) => {
    if (!fitKey || !mapWidth.value || !mapHeight.value) return
    if (fitKey === lastFitKey && viewportReady.value) return
    await nextTick()
    fitTo(props.autoFitMode)
    lastFitKey = fitKey
  },
  { immediate: true }
)

watchEffect(() => {
  if (!canvasRef.value || !activeMap.value || !activeRaster.value || !stageSize.width || !stageSize.height) return
  drawMap()
})

onMounted(() => {
  if (!stageRef.value) return
  resizeObserver = new ResizeObserver((entries) => {
    const entry = entries[0]
    stageSize.width = Math.round(entry.contentRect.width)
    stageSize.height = Math.round(entry.contentRect.height)
    if (!viewportReady.value && mapWidth.value && mapHeight.value) {
      fitTo(props.autoFitMode)
    } else {
      clampViewport()
      drawMap()
    }
  })
  resizeObserver.observe(stageRef.value)
})

onBeforeUnmount(() => {
  resizeObserver?.disconnect()
})

defineExpose({
  fitToContent: () => fitTo('content'),
  fitToMap: () => fitTo('full'),
  zoomIn: () => zoomAt(stageSize.width / 2, stageSize.height / 2, 1.2),
  zoomOut: () => zoomAt(stageSize.width / 2, stageSize.height / 2, 1 / 1.2)
})

function parsePgm(buffer) {
  const bytes = new Uint8Array(buffer)
  let offset = 0

  function skipSpaceAndComments() {
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset])
      if (/\s/.test(ch)) {
        offset += 1
        continue
      }
      if (ch === '#') {
        while (offset < bytes.length && bytes[offset] !== 10 && bytes[offset] !== 13) {
          offset += 1
        }
        continue
      }
      break
    }
  }

  function nextToken() {
    skipSpaceAndComments()
    const start = offset
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset])
      if (/\s/.test(ch) || ch === '#') break
      offset += 1
    }
    return new TextDecoder('ascii').decode(bytes.slice(start, offset))
  }

  const magic = nextToken()
  const width = Number(nextToken())
  const height = Number(nextToken())
  const maxValue = Number(nextToken())
  skipSpaceAndComments()

  if (magic !== 'P5') {
    throw new Error(`Unsupported PGM format: ${magic}`)
  }
  if (!Number.isFinite(width) || !Number.isFinite(height) || !Number.isFinite(maxValue)) {
    throw new Error('Invalid PGM header')
  }

  const pixelCount = width * height
  const grayscale = bytes.slice(offset, offset + pixelCount)
  if (grayscale.length !== pixelCount) {
    throw new Error('PGM data is incomplete')
  }

  const imageData = new ImageData(width, height)
  for (let i = 0; i < pixelCount; i += 1) {
    const value = grayscale[i]
    const base = i * 4
    imageData.data[base] = value
    imageData.data[base + 1] = value
    imageData.data[base + 2] = value
    imageData.data[base + 3] = 255
  }

  const offscreen = document.createElement('canvas')
  offscreen.width = width
  offscreen.height = height
  offscreen.getContext('2d').putImageData(imageData, 0, 0)

  return { width, height, canvas: offscreen }
}

function getCanvasContext() {
  if (!canvasRef.value || !stageSize.width || !stageSize.height) return null
  const dpr = window.devicePixelRatio || 1
  const canvasWidth = Math.max(1, Math.round(stageSize.width * dpr))
  const canvasHeight = Math.max(1, Math.round(stageSize.height * dpr))
  if (canvasRef.value.width !== canvasWidth) canvasRef.value.width = canvasWidth
  if (canvasRef.value.height !== canvasHeight) canvasRef.value.height = canvasHeight
  const ctx = canvasRef.value.getContext('2d')
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, stageSize.width, stageSize.height)
  return ctx
}

function drawMap() {
  const ctx = getCanvasContext()
  if (!ctx || !activeRaster.value) return

  ctx.fillStyle = '#07162f'
  ctx.fillRect(0, 0, stageSize.width, stageSize.height)

  ctx.save()
  applyViewportTransform(ctx)
  ctx.drawImage(activeRaster.value.canvas, 0, 0, mapWidth.value, mapHeight.value)
  drawMapGrid(ctx)
  ctx.restore()

  if (props.visibleLayers?.racks !== false) drawRacks(ctx)
  if (props.visibleLayers?.anchors !== false) drawAnchors(ctx)
  if (props.visibleLayers?.slots !== false) drawSlots(ctx)
  if (props.visibleLayers?.robots && props.robots?.length) drawRobots(ctx)
}

function applyViewportTransform(ctx) {
  ctx.translate(stageSize.width / 2, stageSize.height / 2)
  ctx.scale(viewport.scale, viewport.scale)
  ctx.translate(-viewport.centerX, -viewport.centerY)
}

function drawMapGrid(ctx) {
  if (viewport.scale < 0.18) return
  ctx.save()
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.18)'
  ctx.lineWidth = 1 / viewport.scale
  const step = 50
  for (let x = 0; x < mapWidth.value; x += step) {
    ctx.beginPath()
    ctx.moveTo(x, 0)
    ctx.lineTo(x, mapHeight.value)
    ctx.stroke()
  }
  for (let y = 0; y < mapHeight.value; y += step) {
    ctx.beginPath()
    ctx.moveTo(0, y)
    ctx.lineTo(mapWidth.value, y)
    ctx.stroke()
  }
  ctx.restore()
}

function drawAnchors(ctx) {
  for (const anchor of props.bundle?.anchors || []) {
    const pixel = mapToPixel(anchor.x, anchor.y)
    const screen = pixelToScreen(pixel.x, pixel.y)
    const isSelected = props.selectedKey === `anchor:${anchor.id}`
    const radius = isSelected ? 10 : 7
    const length = isSelected ? 28 : 20

    ctx.save()
    ctx.translate(screen.x, screen.y)
    ctx.rotate(-(Number(anchor.yaw) || 0))
    ctx.strokeStyle = isSelected ? '#facc15' : '#22d3ee'
    ctx.fillStyle = isSelected ? '#facc15' : '#67e8f9'
    ctx.lineWidth = isSelected ? 3 : 2
    ctx.beginPath()
    ctx.arc(0, 0, radius, 0, Math.PI * 2)
    ctx.fill()
    ctx.beginPath()
    ctx.moveTo(0, 0)
    ctx.lineTo(length, 0)
    ctx.stroke()
    ctx.beginPath()
    ctx.moveTo(length, 0)
    ctx.lineTo(length - 7, -4)
    ctx.lineTo(length - 7, 4)
    ctx.closePath()
    ctx.fill()
    ctx.restore()

    drawLabel(ctx, anchor.label, screen.x + 12, screen.y - 10, isSelected ? '#fde68a' : '#a5f3fc')
  }
}

function drawRacks(ctx) {
  const resolution = Number(activeMap.value?.resolution || 0.03)
  for (const rack of props.bundle?.racks || []) {
    const centerPixel = mapToPixel(rack.x, rack.y)
    const center = pixelToScreen(centerPixel.x, centerPixel.y)
    const widthPx = Math.max(18, Number(rack.width || 0.8) / resolution) * viewport.scale
    const depthPx = Math.max(12, Number(rack.depth || 0.4) / resolution) * viewport.scale
    const isSelected = props.selectedKey === `rack:${rack.id}`

    ctx.save()
    ctx.translate(center.x, center.y)
    ctx.rotate(-(Number(rack.yaw) || 0))
    ctx.fillStyle = isSelected ? 'rgba(250, 204, 21, 0.30)' : 'rgba(249, 115, 22, 0.22)'
    ctx.strokeStyle = isSelected ? '#facc15' : '#fb923c'
    ctx.lineWidth = isSelected ? 3 : 2
    ctx.beginPath()
    ctx.rect(-widthPx / 2, -depthPx / 2, widthPx, depthPx)
    ctx.fill()
    ctx.stroke()
    ctx.restore()

    drawLabel(ctx, rack.label, center.x + 10, center.y + 16, isSelected ? '#fde68a' : '#fdba74')
  }
}

function drawSlots(ctx) {
  for (const slot of props.bundle?.slots || []) {
    const pose = slot.nav_pose || slot.service_pose || { x: 0, y: 0 }
    const pixel = mapToPixel(pose.x, pose.y)
    const screen = pixelToScreen(pixel.x, pixel.y)
    const isSelected = props.selectedKey === `slot:${slot.id}`
    const radius = isSelected ? 8 : 5

    ctx.save()
    ctx.fillStyle = isSelected ? '#86efac' : '#22c55e'
    ctx.strokeStyle = isSelected ? '#f8fafc' : '#dcfce7'
    ctx.lineWidth = isSelected ? 3 : 1.5
    ctx.beginPath()
    ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
    ctx.restore()

    if (isSelected) {
      drawLabel(ctx, slot.label, screen.x + 12, screen.y - 12, '#bbf7d0')
    }
  }
}

function drawRobots(ctx) {
  for (const robot of props.robots || []) {
    if (robot.x == null || robot.y == null) continue
    const pixel = mapToPixel(robot.x, robot.y)
    const screen = pixelToScreen(pixel.x, pixel.y)
    const isSelected = props.selectedKey === `robot:${robot.robot_number}`

    ctx.save()
    ctx.translate(screen.x, screen.y)
    ctx.rotate(-(Number(robot.yaw) || 0))
    ctx.fillStyle = isSelected ? '#f472b6' : '#60a5fa'
    ctx.strokeStyle = '#f8fafc'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.arc(0, 0, isSelected ? 11 : 9, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
    ctx.beginPath()
    ctx.moveTo(0, 0)
    ctx.lineTo(18, 0)
    ctx.stroke()
    ctx.restore()

    drawLabel(ctx, robot.robot_number, screen.x + 14, screen.y + 18, isSelected ? '#fbcfe8' : '#bfdbfe')
  }
}

function drawLabel(ctx, text, x, y, color) {
  ctx.save()
  ctx.font = '12px "Segoe UI", sans-serif'
  ctx.fillStyle = 'rgba(3, 7, 18, 0.78)'
  const width = ctx.measureText(text).width
  ctx.fillRect(x - 4, y - 12, width + 8, 16)
  ctx.fillStyle = color
  ctx.fillText(text, x, y)
  ctx.restore()
}

function mapToPixel(x, y) {
  const resolution = Number(activeMap.value?.resolution || 1)
  const originX = Number(activeMap.value?.origin?.[0] || 0)
  const originY = Number(activeMap.value?.origin?.[1] || 0)
  return {
    x: (Number(x) - originX) / resolution,
    y: mapHeight.value - (Number(y) - originY) / resolution
  }
}

function pixelToMap(x, y) {
  const resolution = Number(activeMap.value?.resolution || 1)
  const originX = Number(activeMap.value?.origin?.[0] || 0)
  const originY = Number(activeMap.value?.origin?.[1] || 0)
  return {
    x: originX + x * resolution,
    y: originY + (mapHeight.value - y) * resolution
  }
}

function pixelToScreen(x, y) {
  return {
    x: (x - viewport.centerX) * viewport.scale + stageSize.width / 2,
    y: (y - viewport.centerY) * viewport.scale + stageSize.height / 2
  }
}

function screenToPixel(x, y) {
  return {
    x: viewport.centerX + (x - stageSize.width / 2) / viewport.scale,
    y: viewport.centerY + (y - stageSize.height / 2) / viewport.scale
  }
}

function screenToMap(x, y) {
  const pixel = screenToPixel(x, y)
  return pixelToMap(pixel.x, pixel.y)
}

function fitTo(mode = 'content') {
  if (!stageSize.width || !stageSize.height || !mapWidth.value || !mapHeight.value) return

  const bounds = mode === 'full'
    ? { minX: 0, minY: 0, maxX: mapWidth.value, maxY: mapHeight.value }
    : getContentBounds()

  const width = Math.max(1, bounds.maxX - bounds.minX)
  const height = Math.max(1, bounds.maxY - bounds.minY)
  const margin = 42
  const scaleX = Math.max(0.1, (stageSize.width - margin * 2) / width)
  const scaleY = Math.max(0.1, (stageSize.height - margin * 2) / height)

  viewport.scale = clamp(Math.min(scaleX, scaleY), 0.14, 8)
  viewport.centerX = bounds.minX + width / 2
  viewport.centerY = bounds.minY + height / 2
  viewport.mode = mode
  clampViewport()
  viewportReady.value = true
  drawMap()
}

function getContentBounds() {
  const points = []
  const resolution = Number(activeMap.value?.resolution || 0.03)

  for (const anchor of props.bundle?.anchors || []) {
    const pixel = mapToPixel(anchor.x, anchor.y)
    points.push({ x: pixel.x, y: pixel.y })
  }

  for (const slot of props.bundle?.slots || []) {
    const pose = slot.nav_pose || slot.service_pose
    if (!pose) continue
    const pixel = mapToPixel(pose.x, pose.y)
    points.push({ x: pixel.x, y: pixel.y })
  }

  for (const rack of props.bundle?.racks || []) {
    const center = mapToPixel(rack.x, rack.y)
    const halfWidth = Math.max(18, Number(rack.width || 0.8) / resolution) / 2
    const halfDepth = Math.max(12, Number(rack.depth || 0.4) / resolution) / 2
    const yaw = Number(rack.yaw || 0)
    const corners = [
      { x: -halfWidth, y: -halfDepth },
      { x: halfWidth, y: -halfDepth },
      { x: halfWidth, y: halfDepth },
      { x: -halfWidth, y: halfDepth }
    ].map((corner) => rotatePoint(corner.x, corner.y, -yaw))

    for (const corner of corners) {
      points.push({ x: center.x + corner.x, y: center.y + corner.y })
    }
  }

  for (const robot of props.robots || []) {
    if (robot.x == null || robot.y == null) continue
    const pixel = mapToPixel(robot.x, robot.y)
    points.push({ x: pixel.x, y: pixel.y })
  }

  if (!points.length) {
    return { minX: 0, minY: 0, maxX: mapWidth.value, maxY: mapHeight.value }
  }

  const xs = points.map((point) => point.x)
  const ys = points.map((point) => point.y)
  const pad = 36
  return {
    minX: clamp(Math.min(...xs) - pad, 0, mapWidth.value),
    minY: clamp(Math.min(...ys) - pad, 0, mapHeight.value),
    maxX: clamp(Math.max(...xs) + pad, 0, mapWidth.value),
    maxY: clamp(Math.max(...ys) + pad, 0, mapHeight.value)
  }
}

function clampViewport() {
  if (!mapWidth.value || !mapHeight.value || !stageSize.width || !stageSize.height || !viewport.scale) return
  const halfVisibleWidth = stageSize.width / (2 * viewport.scale)
  const halfVisibleHeight = stageSize.height / (2 * viewport.scale)
  const padX = Math.min(mapWidth.value * 0.35, halfVisibleWidth)
  const padY = Math.min(mapHeight.value * 0.35, halfVisibleHeight)

  viewport.centerX = clamp(
    viewport.centerX,
    -padX + halfVisibleWidth * 0.3,
    mapWidth.value + padX - halfVisibleWidth * 0.3
  )
  viewport.centerY = clamp(
    viewport.centerY,
    -padY + halfVisibleHeight * 0.3,
    mapHeight.value + padY - halfVisibleHeight * 0.3
  )
}

function handleCanvasClick(event) {
  if (!props.interactive || !props.bundle || suppressClick) {
    suppressClick = false
    return
  }

  const point = getLocalPoint(event)
  const pixel = screenToPixel(point.x, point.y)
  const entity = pickEntity(pixel.x, pixel.y)
  if (entity) emit('select', entity)
}

function handlePointerDown(event) {
  if (!props.interactive || !canvasRef.value) return

  const point = getLocalPoint(event)
  const pixel = screenToPixel(point.x, point.y)
  const entity = pickEntity(pixel.x, pixel.y)
  const canDragEntity = props.editable && entity && props.draggableTypes.includes(entity.type)

  if (canDragEntity) {
    const pose = getEntityPose(entity)
    const localMap = screenToMap(point.x, point.y)
    dragState = {
      kind: 'entity',
      pointerId: event.pointerId,
      entity,
      moved: false,
      offsetX: pose.x - localMap.x,
      offsetY: pose.y - localMap.y
    }
    emit('select', entity)
  } else {
    dragState = {
      kind: 'pan',
      pointerId: event.pointerId,
      moved: false,
      lastX: point.x,
      lastY: point.y
    }
  }

  canvasRef.value.setPointerCapture?.(event.pointerId)
}

function handlePointerMove(event) {
  if (!dragState || dragState.pointerId !== event.pointerId) return
  const point = getLocalPoint(event)

  if (dragState.kind === 'pan') {
    const dx = point.x - dragState.lastX
    const dy = point.y - dragState.lastY
    if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
      dragState.moved = true
      suppressClick = true
    }
    viewport.centerX -= dx / viewport.scale
    viewport.centerY -= dy / viewport.scale
    dragState.lastX = point.x
    dragState.lastY = point.y
    clampViewport()
    drawMap()
    return
  }

  const mapPose = screenToMap(point.x, point.y)
  const nextX = mapPose.x + dragState.offsetX
  const nextY = mapPose.y + dragState.offsetY
  dragState.moved = true
  suppressClick = true
  viewport.mode = 'content'
  emit('move-entity', {
    ...dragState.entity,
    x: nextX,
    y: nextY
  })
}

function handlePointerUp(event) {
  if (!dragState || dragState.pointerId !== event.pointerId) return
  canvasRef.value?.releasePointerCapture?.(event.pointerId)
  dragState = null
}

function handleWheel(event) {
  if (!props.interactive) return
  const point = getLocalPoint(event)
  const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12
  zoomAt(point.x, point.y, factor)
}

function zoomAt(screenX, screenY, factor) {
  const oldScale = viewport.scale || 1
  const newScale = clamp(oldScale * factor, 0.14, 8)
  if (newScale === oldScale) return

  const pixelBefore = screenToPixel(screenX, screenY)
  viewport.scale = newScale
  viewport.centerX = pixelBefore.x - (screenX - stageSize.width / 2) / newScale
  viewport.centerY = pixelBefore.y - (screenY - stageSize.height / 2) / newScale
  viewport.mode = 'content'
  viewportReady.value = true
  clampViewport()
  drawMap()
}

function getLocalPoint(event) {
  const rect = canvasRef.value.getBoundingClientRect()
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top
  }
}

function pickEntity(pixelX, pixelY) {
  const candidates = []

  if (props.visibleLayers?.robots) {
    for (const robot of props.robots || []) {
      if (robot.x == null || robot.y == null) continue
      const point = mapToPixel(robot.x, robot.y)
      candidates.push({
        key: `robot:${robot.robot_number}`,
        type: 'robot',
        label: robot.robot_number,
        priority: 0,
        distance: Math.hypot(point.x - pixelX, point.y - pixelY),
        threshold: 18 / Math.max(viewport.scale, 0.3),
        ...robot
      })
    }
  }

  if (props.visibleLayers?.slots !== false) {
    for (const slot of props.bundle?.slots || []) {
      const pose = slot.nav_pose || slot.service_pose || { x: 0, y: 0 }
      const point = mapToPixel(pose.x, pose.y)
      candidates.push({
        key: `slot:${slot.id}`,
        type: 'slot',
        label: slot.label,
        priority: 1,
        distance: Math.hypot(point.x - pixelX, point.y - pixelY),
        threshold: 20 / Math.max(viewport.scale, 0.3),
        ...slot
      })
    }
  }

  if (props.visibleLayers?.anchors !== false) {
    for (const anchor of props.bundle?.anchors || []) {
      const point = mapToPixel(anchor.x, anchor.y)
      candidates.push({
        key: `anchor:${anchor.id}`,
        type: 'anchor',
        label: anchor.label,
        priority: 2,
        distance: Math.hypot(point.x - pixelX, point.y - pixelY),
        threshold: 24 / Math.max(viewport.scale, 0.3),
        ...anchor
      })
    }
  }

  if (props.visibleLayers?.racks !== false) {
    for (const rack of props.bundle?.racks || []) {
      const point = mapToPixel(rack.x, rack.y)
      const resolution = Number(activeMap.value?.resolution || 0.03)
      const halfWidth = Math.max(18, Number(rack.width || 0.8) / resolution) / 2
      const halfDepth = Math.max(12, Number(rack.depth || 0.4) / resolution) / 2
      const local = rotatePoint(pixelX - point.x, pixelY - point.y, Number(rack.yaw || 0))
      const inside = Math.abs(local.x) <= halfWidth && Math.abs(local.y) <= halfDepth
      candidates.push({
        key: `rack:${rack.id}`,
        type: 'rack',
        label: rack.label,
        priority: inside ? 2.5 : 3,
        distance: inside ? 0 : Math.hypot(point.x - pixelX, point.y - pixelY),
        threshold: inside ? Infinity : 34 / Math.max(viewport.scale, 0.3),
        ...rack
      })
    }
  }

  return candidates
    .filter((item) => item.distance <= item.threshold)
    .sort((a, b) => a.priority - b.priority || a.distance - b.distance)[0] || null
}

function getEntityPose(entity) {
  if (entity.type === 'slot') {
    return entity.nav_pose || entity.service_pose || { x: 0, y: 0 }
  }
  return {
    x: Number(entity.x || 0),
    y: Number(entity.y || 0)
  }
}

function rotatePoint(x, y, yaw) {
  const cos = Math.cos(yaw)
  const sin = Math.sin(yaw)
  return {
    x: x * cos - y * sin,
    y: x * sin + y * cos
  }
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value))
}
</script>
