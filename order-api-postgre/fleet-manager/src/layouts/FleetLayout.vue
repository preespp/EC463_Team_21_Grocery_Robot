<template>
  <div class="fleet-shell">
    <aside class="sidebar">
      <div class="brand">{{ t('brand') }}</div>
      <nav>
        <RouterLink to="/app/inventory-report"><span class="nav-ico">📊</span>{{ t('nav_inventory') }}</RouterLink>
        <RouterLink to="/app/ask-ai"><span class="nav-ico">✨</span>{{ t('nav_ask_ai') }}</RouterLink>
        <RouterLink to="/app/location-map"><span class="nav-ico">🗺️</span>{{ t('nav_store_map') }}</RouterLink>
        <RouterLink to="/app/slam-map"><span class="nav-ico">📡</span>{{ t('nav_slam') }}</RouterLink>
        <RouterLink to="/app/robot-status"><span class="nav-ico">🤖</span>{{ t('nav_robot_status') }}</RouterLink>
        <RouterLink to="/app/restock"><span class="nav-ico">📦</span>{{ t('nav_restock') }}</RouterLink>
        <RouterLink to="/app/employee-accounts"><span class="nav-ico">👤</span>{{ t('nav_employee_accounts') }}</RouterLink>
      </nav>
      <button class="btn ghost" @click="logout">{{ t('logout') }}</button>
    </aside>

    <main class="content">
      <header class="topbar">
        <div class="top-left">
          <img class="topbar-logo" src="/full_logo.png" alt="GOFR logo" />
          <input class="search" :placeholder="t('search_placeholder')" />
        </div>
        <div class="top-right">
          <div class="lang-switch">
            <button class="lang-btn" :class="{ active: currentLang === 'en' }" @click="setLang('en')" title="English">
              <img class="flag-icon" :src="'/USA.png'" alt="English" />
            </button>
            <button class="lang-btn" :class="{ active: currentLang === 'es' }" @click="setLang('es')" title="Español">
              <img class="flag-icon" :src="'/Spanish.png'" alt="Español" />
            </button>
            <button class="lang-btn" :class="{ active: currentLang === 'zh' }" @click="setLang('zh')" title="中文">
              <img class="flag-icon" :src="'/Mandarin.png'" alt="中文" />
            </button>
          </div>
          <button class="btn ghost help-btn" @click="showHelp = true">{{ t('help') }}</button>
          <div class="who"><span class="profile-icon">👨‍💼</span>{{ employeeName }}</div>
        </div>
      </header>
      <section class="view-wrap">
        <RouterView />
      </section>
    </main>
  </div>

  <div v-if="showHelp" class="help-overlay" @click.self="showHelp = false">
    <div class="help-card">
      <h3>{{ t('help_title') }}</h3>
      <ul>
        <li>{{ t('help_1') }}</li>
        <li>{{ t('help_2') }}</li>
        <li>{{ t('help_3') }}</li>
        <li>{{ t('help_4') }}</li>
      </ul>
      <button class="btn" @click="showHelp = false">{{ t('close') }}</button>
    </div>
  </div>
</template>

<script setup>
import { computed, ref } from 'vue'
import { RouterLink, RouterView, useRouter } from 'vue-router'
import { currentLang, setLang as setLangValue, t } from '../i18n'

const router = useRouter()
const showHelp = ref(false)

const employeeName = computed(() => {
  const first = localStorage.getItem('fleet_first_name') || ''
  const last = localStorage.getItem('fleet_last_name') || ''
  const id = localStorage.getItem('fleet_employee_id') || ''
  const full = `${first} ${last}`.trim()
  return full ? `${full} (${id})` : id
})

function setLang(lang) {
  setLangValue(lang)
}

function logout() {
  localStorage.removeItem('fleet_token')
  localStorage.removeItem('fleet_employee_id')
  localStorage.removeItem('fleet_first_name')
  localStorage.removeItem('fleet_last_name')
  router.push('/login')
}
</script>
