<template>
  <div class="map-page">
    <div class="cards-grid map-summary-grid">
      <div class="metric-card">
        <div class="label">Viewer</div>
        <div class="value map-metric-value">ROS Web GUI</div>
      </div>
      <div class="metric-card">
        <div class="label">Transport</div>
        <div class="value map-metric-value">rosbridge websocket</div>
      </div>
      <div class="metric-card">
        <div class="label">Default Bridge</div>
        <div class="value map-metric-value">{{ defaultBridgeUrl }}</div>
      </div>
      <div class="metric-card">
        <div class="label">Embed Mode</div>
        <div class="value map-metric-value">{{ frameLoaded ? 'Ready' : 'Loading' }}</div>
      </div>
    </div>

    <div class="map-workspace mt16">
      <section class="panel map-surface-panel">
        <div class="map-toolbar">
          <div class="map-toolbar-copy">
            <h2>{{ t('slam_map') }}</h2>
            <p class="muted map-subtitle">
              Embedded ROS visualization workspace built from the vendored
              `StarLionJiang/ros_web_gui_app` fork. The full React + Three.js toolset runs
              inside this panel, including map layers, relocalization, manual control,
              topology inspection, and map editing.
            </p>
          </div>

          <div class="map-toolbar-actions">
            <div class="map-action-row">
              <button class="btn small" type="button" @click="reloadEmbeddedApp">
                Reload Panel
              </button>
              <button class="btn ghost small" type="button" @click="openStandaloneApp">
                Open Standalone
              </button>
              <button class="btn ghost small" type="button" @click="openReferenceRepo">
                Open Source Repo
              </button>
            </div>

            <div class="map-status-row">
              <span class="map-status-pill" :class="frameLoaded ? 'clean' : 'dirty'">
                {{ frameLoaded ? 'Embedded App Loaded' : 'Loading Embedded App' }}
              </span>
              <span class="muted tiny">
                The embedded app manages its own rosbridge connection inside the iframe.
              </span>
            </div>
          </div>
        </div>

        <div class="embedded-slam-stage">
          <iframe
            :key="frameKey"
            ref="iframeRef"
            :src="iframeSrc"
            class="embedded-slam-frame"
            title="Embedded ROS Web GUI"
            allow="fullscreen"
            allowfullscreen
            @load="handleFrameLoad"
          />
          <div v-if="!frameLoaded" class="embedded-slam-overlay">
            Initializing embedded ROS Web GUI...
          </div>
        </div>
      </section>

      <aside class="map-side-stack">
        <div class="panel">
          <h3>Bridge Setup</h3>
          <div class="foxglove-command-block">
            <code>sudo apt install ros-$ROS_DISTRO-rosbridge-server</code>
            <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code>
          </div>
          <div class="muted tiny slam-help-copy">
            Inside the embedded app, connect to <code>{{ defaultBridgeUrl }}</code>. The
            connection page defaults to the current hostname and port <code>9090</code>.
          </div>
        </div>

        <div class="panel">
          <h3>Included Tools</h3>
          <div class="slam-feature-list">
            <span>2D / 3D map view</span>
            <span>Layer settings</span>
            <span>Manual control</span>
            <span>Relocalize pose</span>
            <span>Map editor</span>
            <span>Topology info</span>
            <span>Image overlays</span>
            <span>Topic-driven layers</span>
          </div>
          <div class="muted tiny slam-help-copy">
            This is the reference app itself running as a child app, not a visual imitation of it.
          </div>
        </div>

        <div class="panel map-inspector">
          <h3>Integration Notes</h3>
          <div class="map-kv-grid">
            <div>
              <span class="muted tiny">Embedded URL</span>
              <strong>{{ embeddedAppPath }}</strong>
            </div>
            <div>
              <span class="muted tiny">Reference Repo</span>
              <strong>StarLionJiang/ros_web_gui_app</strong>
            </div>
            <div>
              <span class="muted tiny">Connection Style</span>
              <strong>rosbridge in iframe</strong>
            </div>
            <div>
              <span class="muted tiny">Vendored Source</span>
              <strong>/third_party/ros_web_gui_app</strong>
            </div>
          </div>
          <div class="muted tiny slam-help-copy">
            The embedded build is regenerated from the vendored source and served by the main Vue
            app from <code>/embedded/ros-web-gui/index.html</code>.
          </div>
        </div>
      </aside>
    </div>
  </div>
</template>

<script setup>
import { computed, ref } from 'vue'
import { t } from '../i18n'

const iframeRef = ref(null)
const frameKey = ref(0)
const frameLoaded = ref(false)

const embeddedAppPath = '/embedded/ros-web-gui/index.html'
const sourceRepoUrl = 'https://github.com/StarLionJiang/ros_web_gui_app'

const iframeSrc = computed(() => `${embeddedAppPath}?embed=1&v=${frameKey.value}`)
const defaultBridgeUrl = computed(() => {
  const host = window.location.hostname || 'localhost'
  return `ws://${host}:9090`
})

function handleFrameLoad() {
  frameLoaded.value = true
}

function reloadEmbeddedApp() {
  frameLoaded.value = false
  frameKey.value += 1
}

function openStandaloneApp() {
  window.open(iframeSrc.value, '_blank', 'noopener,noreferrer')
}

function openReferenceRepo() {
  window.open(sourceRepoUrl, '_blank', 'noopener,noreferrer')
}
</script>
