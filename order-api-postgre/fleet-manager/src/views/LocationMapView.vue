<template>
  <div class="map-page">
    <div class="cards-grid map-summary-grid">
      <div class="metric-card">
        <div class="label">Semantic Map</div>
        <div class="value map-metric-value">{{ bundle?.map?.semantic_id || '--' }}</div>
      </div>
      <div class="metric-card">
        <div class="label">DB Version</div>
        <div class="value map-metric-value">v{{ versionInfo?.version_seq ?? '--' }}</div>
      </div>
      <div class="metric-card">
        <div class="label">Anchors</div>
        <div class="value">{{ bundle?.summary?.anchor_count ?? 0 }}</div>
      </div>
      <div class="metric-card">
        <div class="label">Racks</div>
        <div class="value">{{ bundle?.summary?.rack_count ?? 0 }}</div>
      </div>
      <div class="metric-card">
        <div class="label">Slots</div>
        <div class="value">{{ bundle?.summary?.slot_count ?? 0 }}</div>
      </div>
    </div>

    <div v-if="error" class="panel mt16 map-alert">{{ error }}</div>
    <div v-if="saveMessage" class="panel mt16 map-success">{{ saveMessage }}</div>

    <div class="map-workspace mt16">
      <section class="panel map-surface-panel">
        <div class="map-toolbar">
          <div class="map-toolbar-copy">
            <h2>{{ t('store_map') }}</h2>
            <p class="muted map-subtitle">
              Fit to the active store layout, zoom with the wheel, drag the background to pan,
              and use edit mode to reposition anchors, racks, and slots before saving.
            </p>
          </div>

          <div class="map-toolbar-actions">
            <div class="map-chip-row">
              <label class="map-chip">
                <input v-model="layers.anchors" type="checkbox" />
                Anchors
              </label>
              <label class="map-chip">
                <input v-model="layers.racks" type="checkbox" />
                Racks
              </label>
              <label class="map-chip">
                <input v-model="layers.slots" type="checkbox" />
                Slots
              </label>
            </div>

            <div class="map-action-row">
              <button class="btn ghost small" type="button" @click="canvasRef?.fitToContent()">
                Fit Objects
              </button>
              <button class="btn ghost small" type="button" @click="canvasRef?.fitToMap()">
                Fit Map
              </button>
              <button class="btn ghost small" type="button" @click="canvasRef?.zoomIn()">+</button>
              <button class="btn ghost small" type="button" @click="canvasRef?.zoomOut()">-</button>
              <button
                class="btn small"
                :class="{ ghost: !editMode }"
                type="button"
                @click="editMode = !editMode"
              >
                {{ editMode ? 'Editing' : 'Edit Mode' }}
              </button>
              <button class="btn ghost small" type="button" :disabled="!dirty" @click="resetDraft">
                Reset
              </button>
              <button class="btn small" type="button" :disabled="!dirty || saving" @click="saveDraft">
                {{ saving ? 'Saving...' : 'Save' }}
              </button>
            </div>

            <div class="map-status-row">
              <span class="map-status-pill" :class="dirty ? 'dirty' : 'clean'">
                {{ dirty ? 'Unsaved changes' : 'Saved' }}
              </span>
              <span class="muted tiny">
                {{ editMode ? 'Drag selected map objects to update the semantic overlay.' : 'Enable edit mode to drag points.' }}
              </span>
            </div>
          </div>
        </div>

        <SemanticMapCanvas
          ref="canvasRef"
          :bundle="bundle"
          :robots="[]"
          :visible-layers="layers"
          :selected-key="selectedKey"
          :editable="editMode"
          auto-fit-mode="content"
          @select="selectEntity"
          @move-entity="moveEntity"
        />
      </section>

      <aside class="map-side-stack">
        <div class="panel map-inspector">
          <h3>Selected Object</h3>
          <template v-if="selected">
            <div class="map-badge-row">
              <span class="map-entity-badge">{{ selected.type }}</span>
              <span class="muted">{{ selected.label }}</span>
            </div>
            <div class="map-kv-grid">
              <div>
                <span class="muted tiny">ID</span>
                <strong>{{ selected.id || selected.robot_number || '--' }}</strong>
              </div>
              <div>
                <span class="muted tiny">Anchor</span>
                <strong>{{ selected.anchor_id || '--' }}</strong>
              </div>
              <div>
                <span class="muted tiny">Rack</span>
                <strong>{{ selected.rack_id || '--' }}</strong>
              </div>
              <div>
                <span class="muted tiny">Level</span>
                <strong>{{ selected.rack_level ?? '--' }}</strong>
              </div>
            </div>
            <div class="map-pose-box">
              <div><span class="muted tiny">x</span> {{ formatPoseValue(displayPose?.x) }}</div>
              <div><span class="muted tiny">y</span> {{ formatPoseValue(displayPose?.y) }}</div>
              <div><span class="muted tiny">z</span> {{ formatPoseValue(displayPose?.z) }}</div>
              <div><span class="muted tiny">yaw</span> {{ formatPoseValue(displayPose?.yaw) }}</div>
            </div>
            <div v-if="versionInfo" class="map-version-box">
              <div><span class="muted tiny">Version</span> v{{ versionInfo.version_seq }}</div>
              <div><span class="muted tiny">Saved At</span> {{ formatTimestamp(versionInfo.saved_at) }}</div>
              <div><span class="muted tiny">Saved By</span> {{ versionInfo.saved_by_employee_id || 'System' }}</div>
              <div><span class="muted tiny">Summary</span> {{ versionInfo.change_summary || '--' }}</div>
            </div>
            <div v-if="editMode && canEditSelected" class="map-edit-panel">
              <div class="map-section-title">Manual Pose Edit</div>
              <div class="map-edit-grid">
                <label class="field">
                  <span class="muted tiny">X</span>
                  <input v-model="poseEditor.x" type="number" step="0.01" />
                </label>
                <label class="field">
                  <span class="muted tiny">Y</span>
                  <input v-model="poseEditor.y" type="number" step="0.01" />
                </label>
                <label v-if="manualZVisible" class="field">
                  <span class="muted tiny">Z</span>
                  <input v-model="poseEditor.z" type="number" step="0.01" />
                </label>
                <label class="field">
                  <span class="muted tiny">Yaw</span>
                  <input v-model="poseEditor.yaw" type="number" step="0.01" />
                </label>
              </div>
              <div v-if="editorError" class="error tiny">{{ editorError }}</div>
              <div class="map-action-row">
                <button class="btn small" type="button" @click="applyManualPose">Apply Values</button>
                <button class="btn ghost small" type="button" @click="syncPoseEditor">Use Current</button>
              </div>
            </div>
            <div v-if="selectedProducts.length" class="map-product-list">
              <div class="map-section-title">Products In Slot</div>
              <div
                v-for="product in selectedProducts"
                :key="`${selected.key}-${product.product_id}`"
                class="map-product-card"
              >
                <div>
                  <strong>{{ product.product_name || `Product ${product.product_id}` }}</strong>
                  <div class="muted tiny">ID {{ product.product_id }}</div>
                </div>
                <div class="map-product-meta">
                  <span>Stock {{ product.stock ?? '--' }}</span>
                  <span>{{ product.category_name || 'Uncategorized' }}</span>
                </div>
              </div>
            </div>
          </template>
          <p v-else class="muted">Select an anchor, rack, or slot on the map.</p>
        </div>

        <div class="panel">
          <h3>Semantic Slots</h3>
          <div class="table-scroll semantic-table-scroll">
            <table class="semantic-table">
              <thead>
                <tr>
                  <th>Slot</th>
                  <th>Product</th>
                  <th>Stock</th>
                  <th>Level</th>
                </tr>
              </thead>
              <tbody>
                <tr
                  v-for="slot in slots"
                  :key="slot.id"
                  :class="{ 'semantic-row-active': selectedKey === `slot:${slot.id}` }"
                  @click="selectEntity(toEntity('slot', slot))"
                >
                  <td>{{ slot.label }}</td>
                  <td>{{ slot.products?.[0]?.product_name || slot.product_names?.[0] || '--' }}</td>
                  <td>{{ slot.products?.[0]?.stock ?? '--' }}</td>
                  <td>{{ slot.rack_level }}</td>
                </tr>
              </tbody>
            </table>
          </div>
        </div>
      </aside>
    </div>
  </div>
</template>

<script setup>
import { computed, onMounted, reactive, ref } from 'vue'
import { api } from '../api'
import { t } from '../i18n'
import SemanticMapCanvas from '../components/SemanticMapCanvas.vue'

const canvasRef = ref(null)
const bundle = ref(null)
const savedBundle = ref(null)
const error = ref('')
const saveMessage = ref('')
const saving = ref(false)
const selected = ref(null)
const dirty = ref(false)
const editMode = ref(false)
const editorError = ref('')
const layers = reactive({
  anchors: true,
  racks: true,
  slots: true
})
const poseEditor = reactive({
  x: '',
  y: '',
  z: '',
  yaw: ''
})

const slots = computed(() => bundle.value?.slots || [])
const selectedKey = computed(() => selected.value?.key || '')
const selectedProducts = computed(() => selected.value?.products || [])
const displayPose = computed(() => getDisplayPose(selected.value))
const versionInfo = computed(() => bundle.value?.version || null)
const canEditSelected = computed(() => ['anchor', 'rack', 'slot'].includes(selected.value?.type || ''))
const manualZVisible = computed(() => selected.value?.type === 'slot')

onMounted(loadSemanticMap)

async function loadSemanticMap() {
  error.value = ''
  saveMessage.value = ''
  editorError.value = ''
  try {
    const data = await api.semanticMap()
    savedBundle.value = cloneBundle(data)
    bundle.value = cloneBundle(data)
    dirty.value = false

    if (selected.value?.key) {
      selected.value = findEntityByKey(bundle.value, selected.value.key)
    } else if (data.slots?.length) {
      selected.value = toEntity('slot', data.slots[0])
    }
    syncPoseEditor()
  } catch (err) {
    error.value = err.message || 'Unable to load semantic map.'
  }
}

function cloneBundle(data) {
  return data ? JSON.parse(JSON.stringify(data)) : null
}

function toEntity(type, raw) {
  if (!raw) return null
  if (raw.key && raw.type) return raw
  const entityId = raw.id || raw.robot_number || raw.anchor_id || raw.rack_id || type
  const label = raw.label || raw.robot_number || raw.id || entityId
  return {
    ...raw,
    key: `${type}:${entityId}`,
    type,
    label
  }
}

function selectEntity(entity) {
  selected.value = toEntity(entity?.type || inferEntityType(entity), entity)
  editorError.value = ''
  syncPoseEditor()
}

function inferEntityType(entity) {
  if (!entity) return 'slot'
  if (entity.robot_number) return 'robot'
  if (entity.products || entity.product_ids) return 'slot'
  if (Object.prototype.hasOwnProperty.call(entity, 'levels')) return 'rack'
  return 'anchor'
}

function getDisplayPose(entity) {
  if (!entity) return null
  if (entity.nav_pose) return entity.nav_pose
  if (entity.service_pose) return entity.service_pose
  if (entity.x != null || entity.y != null) {
    return {
      x: entity.x ?? 0,
      y: entity.y ?? 0,
      z: entity.z ?? 0,
      yaw: entity.yaw ?? 0
    }
  }
  return null
}

function formatPoseValue(value) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(2) : '--'
}

function formatTimestamp(value) {
  if (!value) return '--'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? String(value) : date.toLocaleString()
}

function moveEntity(event) {
  if (!bundle.value || !event?.type) return
  saveMessage.value = ''
  editorError.value = ''

  if (event.type === 'anchor') {
    moveAnchor(event.id, Number(event.x), Number(event.y))
  } else if (event.type === 'rack') {
    moveRack(event.id, Number(event.x), Number(event.y))
  } else if (event.type === 'slot') {
    moveSlot(event.id, Number(event.x), Number(event.y))
  }

  refreshSelection()
  syncPoseEditor()
  dirty.value = true
}

function moveAnchor(anchorId, x, y, yaw = null) {
  const anchor = bundle.value.anchors?.find((item) => item.id === anchorId)
  if (!anchor) return
  const dx = x - Number(anchor.x || 0)
  const dy = y - Number(anchor.y || 0)
  const nextYaw = yaw == null ? Number(anchor.yaw || 0) : Number(yaw)
  const deltaYaw = nextYaw - Number(anchor.yaw || 0)

  anchor.x = x
  anchor.y = y
  anchor.yaw = nextYaw

  for (const rack of bundle.value.racks || []) {
    if (rack.anchor_id !== anchorId) continue
    rack.x = Number(rack.x || 0) + dx
    rack.y = Number(rack.y || 0) + dy
    rack.yaw = Number(rack.yaw || 0) + deltaYaw
  }

  for (const slot of bundle.value.slots || []) {
    if (slot.anchor_id !== anchorId) continue
    shiftSlot(slot, dx, dy, deltaYaw)
  }
}

function moveRack(rackId, x, y, yaw = null) {
  const rack = bundle.value.racks?.find((item) => item.id === rackId)
  if (!rack) return
  const dx = x - Number(rack.x || 0)
  const dy = y - Number(rack.y || 0)
  const nextYaw = yaw == null ? Number(rack.yaw || 0) : Number(yaw)
  const deltaYaw = nextYaw - Number(rack.yaw || 0)

  rack.x = x
  rack.y = y
  rack.yaw = nextYaw

  if (rack.anchor_id) {
    const anchor = bundle.value.anchors?.find((item) => item.id === rack.anchor_id)
    if (anchor) {
      anchor.x = Number(anchor.x || 0) + dx
      anchor.y = Number(anchor.y || 0) + dy
      anchor.yaw = Number(anchor.yaw || 0) + deltaYaw
    }
  }

  for (const slot of bundle.value.slots || []) {
    if (slot.rack_id !== rackId) continue
    shiftSlot(slot, dx, dy, deltaYaw)
  }
}

function moveSlot(slotId, x, y, yaw = null, z = null) {
  const slot = bundle.value.slots?.find((item) => item.id === slotId)
  if (!slot) return
  const pose = slot.nav_pose || slot.service_pose || { x: 0, y: 0 }
  const dx = x - Number(pose.x || 0)
  const dy = y - Number(pose.y || 0)
  const currentYaw = Number(slot.nav_pose?.yaw ?? slot.service_pose?.yaw ?? 0)
  const nextYaw = yaw == null ? currentYaw : Number(yaw)
  const deltaYaw = nextYaw - currentYaw
  shiftSlot(slot, dx, dy, deltaYaw, z)
}

function shiftSlot(slot, dx, dy, deltaYaw = 0, nextZ = null) {
  if (!slot.nav_pose) {
    slot.nav_pose = { x: 0, y: 0, yaw: 0 }
  }
  if (!slot.service_pose) {
    slot.service_pose = { x: slot.nav_pose.x, y: slot.nav_pose.y, z: 0, yaw: slot.nav_pose.yaw || 0 }
  }

  slot.nav_pose.x = Number(slot.nav_pose.x || 0) + dx
  slot.nav_pose.y = Number(slot.nav_pose.y || 0) + dy
  slot.nav_pose.yaw = Number(slot.nav_pose.yaw || 0) + deltaYaw
  slot.service_pose.x = Number(slot.service_pose.x || 0) + dx
  slot.service_pose.y = Number(slot.service_pose.y || 0) + dy
  slot.service_pose.yaw = Number(slot.service_pose.yaw || 0) + deltaYaw
  if (nextZ != null && nextZ !== "") {
    slot.service_pose.z = Number(nextZ)
  }
}

function refreshSelection() {
  if (!selected.value?.key || !bundle.value) return
  selected.value = findEntityByKey(bundle.value, selected.value.key)
  syncPoseEditor()
}

function findEntityByKey(sourceBundle, key) {
  if (!sourceBundle || !key) return null
  const [type, id] = String(key).split(':')
  if (!type || !id) return null

  if (type === 'anchor') {
    return toEntity(type, sourceBundle.anchors?.find((item) => item.id === id))
  }
  if (type === 'rack') {
    return toEntity(type, sourceBundle.racks?.find((item) => item.id === id))
  }
  if (type === 'slot') {
    return toEntity(type, sourceBundle.slots?.find((item) => item.id === id))
  }
  return null
}

function resetDraft() {
  if (!savedBundle.value) return
  bundle.value = cloneBundle(savedBundle.value)
  dirty.value = false
  saveMessage.value = ''
  editorError.value = ''
  refreshSelection()
  canvasRef.value?.fitToContent()
}

async function saveDraft() {
  if (!bundle.value || !dirty.value || saving.value) return
  error.value = ''
  saveMessage.value = ''
  saving.value = true
  try {
    const response = await api.saveSemanticMap(bundle.value, {
      changeSummary: "Saved from Store Map UI"
    })
    const nextBundle = response.bundle || bundle.value
    savedBundle.value = cloneBundle(nextBundle)
    bundle.value = cloneBundle(nextBundle)
    dirty.value = false
    editMode.value = false
    refreshSelection()
    saveMessage.value = `Semantic map saved to the navigation YAML and database version v${nextBundle.version?.version_seq ?? "--"}.`
  } catch (err) {
    error.value = err.message || 'Unable to save semantic map.'
  } finally {
    saving.value = false
  }
}

function syncPoseEditor() {
  const pose = getDisplayPose(selected.value)
  poseEditor.x = formatEditorValue(pose?.x)
  poseEditor.y = formatEditorValue(pose?.y)
  poseEditor.z = manualZVisible.value ? formatEditorValue(selected.value?.service_pose?.z ?? pose?.z) : ''
  poseEditor.yaw = formatEditorValue(pose?.yaw)
}

function formatEditorValue(value) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(3) : ''
}

function applyManualPose() {
  if (!selected.value || !canEditSelected.value) return

  const x = parseEditorNumber(poseEditor.x, 'x')
  const y = parseEditorNumber(poseEditor.y, 'y')
  const yaw = parseEditorNumber(poseEditor.yaw, 'yaw')
  const z = manualZVisible.value ? parseEditorNumber(poseEditor.z, 'z') : null
  if ([x, y, yaw].some((value) => value == null) || (manualZVisible.value && z == null)) {
    return
  }

  editorError.value = ''
  saveMessage.value = ''

  if (selected.value.type === 'anchor') {
    moveAnchor(selected.value.id, x, y, yaw)
  } else if (selected.value.type === 'rack') {
    moveRack(selected.value.id, x, y, yaw)
  } else if (selected.value.type === 'slot') {
    moveSlot(selected.value.id, x, y, yaw, z)
  }

  refreshSelection()
  dirty.value = true
}

function parseEditorNumber(value, label) {
  const parsed = Number(value)
  if (!Number.isFinite(parsed)) {
    editorError.value = `${label} must be a valid number.`
    return null
  }
  return parsed
}
</script>
