import { ref } from 'vue'

const STORAGE_KEY = 'fleet_rosbridge_url'

export function useRosbridgeNav2(options = {}) {
  const defaultUrl = options.defaultUrl || buildDefaultUrl()
  const rosbridgeUrl = ref(readStoredUrl() || defaultUrl)
  const connectionState = ref('disconnected')
  const connectionError = ref('')
  const liveMap = ref(null)
  const robots = ref([])
  const lastUpdate = ref('')

  let socket = null

  function connect() {
    disconnect({ preserveData: true })
    connectionError.value = ''
    connectionState.value = 'connecting'

    socket = new WebSocket(rosbridgeUrl.value)
    socket.addEventListener('open', handleOpen)
    socket.addEventListener('message', handleMessage)
    socket.addEventListener('error', handleError)
    socket.addEventListener('close', handleClose)
    writeStoredUrl(rosbridgeUrl.value)
  }

  function disconnect({ preserveData = false } = {}) {
    if (socket) {
      socket.removeEventListener('open', handleOpen)
      socket.removeEventListener('message', handleMessage)
      socket.removeEventListener('error', handleError)
      socket.removeEventListener('close', handleClose)
      socket.close()
      socket = null
    }

    connectionState.value = 'disconnected'
    if (!preserveData) {
      liveMap.value = null
      robots.value = []
      lastUpdate.value = ''
    }
  }

  function handleOpen() {
    connectionState.value = 'connected'
    connectionError.value = ''
    subscribe('/map', 500)
    subscribe('/tf', 100)
  }

  function handleMessage(event) {
    let data = null
    try {
      data = JSON.parse(event.data)
    } catch (_err) {
      return
    }

    if (data.op === 'error') {
      connectionError.value = data.msg || 'rosbridge returned an error'
      return
    }

    if (data.op !== 'publish') return

    if (data.topic === '/map' && data.msg) {
      liveMap.value = occupancyGridToRaster(data.msg)
      lastUpdate.value = new Date().toLocaleTimeString()
      return
    }

    if (data.topic === '/tf' && data.msg) {
      const nextRobots = extractRobotPoses(data.msg.transforms, options)
      if (nextRobots.length) {
        robots.value = nextRobots
        lastUpdate.value = new Date().toLocaleTimeString()
      }
    }
  }

  function handleError() {
    connectionError.value = 'Unable to reach rosbridge. Start rosbridge_server and verify the WebSocket URL.'
    connectionState.value = 'error'
  }

  function handleClose() {
    if (connectionState.value !== 'error') {
      connectionState.value = 'disconnected'
    }
  }

  function subscribe(topic, throttleRate) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return
    socket.send(JSON.stringify({
      op: 'subscribe',
      topic,
      throttle_rate: throttleRate,
      queue_length: 1
    }))
  }

  return {
    rosbridgeUrl,
    connectionState,
    connectionError,
    liveMap,
    robots,
    lastUpdate,
    connect,
    disconnect
  }
}

function buildDefaultUrl() {
  const host = window.location.hostname || 'localhost'
  return `ws://${host}:9090`
}

function readStoredUrl() {
  try {
    return localStorage.getItem(STORAGE_KEY) || ''
  } catch (_err) {
    return ''
  }
}

function writeStoredUrl(url) {
  try {
    localStorage.setItem(STORAGE_KEY, url)
  } catch (_err) {
    // noop
  }
}

function occupancyGridToRaster(message) {
  const width = Number(message?.info?.width || 0)
  const height = Number(message?.info?.height || 0)
  if (!width || !height) return null

  const data = Array.isArray(message?.data) ? message.data : []
  const canvas = document.createElement('canvas')
  canvas.width = width
  canvas.height = height
  const ctx = canvas.getContext('2d')
  const imageData = ctx.createImageData(width, height)

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const srcIndex = y * width + x
      const dstY = height - 1 - y
      const dstIndex = (dstY * width + x) * 4
      const value = Number(data[srcIndex] ?? -1)
      const shade = value < 0 ? 190 : 255 - Math.round((Math.min(100, Math.max(0, value)) / 100) * 255)

      imageData.data[dstIndex] = shade
      imageData.data[dstIndex + 1] = shade
      imageData.data[dstIndex + 2] = shade
      imageData.data[dstIndex + 3] = 255
    }
  }

  ctx.putImageData(imageData, 0, 0)

  return {
    width,
    height,
    canvas,
    generated_at: new Date().toISOString(),
    image_format: 'occupancy_grid',
    name: 'nav2_live',
    frame_id: message?.header?.frame_id || 'map',
    semantic_id: 'nav2_live',
    resolution: Number(message?.info?.resolution || 0.03),
    origin: [
      Number(message?.info?.origin?.position?.x || 0),
      Number(message?.info?.origin?.position?.y || 0),
      quaternionToYaw(message?.info?.origin?.orientation)
    ],
    width_px: width,
    height_px: height
  }
}

function extractRobotPoses(transforms, options) {
  const mapFrame = normalizeFrame(options.mapFrame || 'map')
  const robotFrame = normalizeFrame(options.robotFrame || 'base_link')
  const matches = []

  for (const transformStamped of Array.isArray(transforms) ? transforms : []) {
    const headerFrame = normalizeFrame(transformStamped?.header?.frame_id)
    const childFrame = normalizeFrame(transformStamped?.child_frame_id)
    if (headerFrame !== mapFrame) continue
    if (!matchesRobotFrame(childFrame, robotFrame)) continue

    const translation = transformStamped?.transform?.translation || {}
    const rotation = transformStamped?.transform?.rotation || {}
    matches.push({
      robot_number: childFrame === robotFrame ? 'RB-01' : childFrame,
      frame_name: childFrame,
      battery_level: null,
      status_key: 'live',
      status_text: 'Live ROS Pose',
      x: Number(translation.x || 0),
      y: Number(translation.y || 0),
      z: Number(translation.z || 0),
      yaw: quaternionToYaw(rotation),
      updated_at: new Date().toISOString()
    })
  }

  return matches
}

function normalizeFrame(value) {
  return String(value || '').replace(/^\//, '')
}

function matchesRobotFrame(childFrame, robotFrame) {
  if (!childFrame || !robotFrame) return false
  return childFrame === robotFrame || childFrame.endsWith(`/${robotFrame}`)
}

function quaternionToYaw(quaternion) {
  const x = Number(quaternion?.x || 0)
  const y = Number(quaternion?.y || 0)
  const z = Number(quaternion?.z || 0)
  const w = Number(quaternion?.w || 1)
  const siny = 2 * (w * z + x * y)
  const cosy = 1 - 2 * (y * y + z * z)
  return Math.atan2(siny, cosy)
}
