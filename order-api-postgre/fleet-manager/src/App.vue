<template>
  <router-view />
</template>

<script setup>
import { watchEffect } from 'vue'
import { useRoute } from 'vue-router'
import { currentLang, t } from './i18n'

const route = useRoute()

const routeTitleKeys = {
  idle: 'idle_title',
  login: 'login_title',
  'inventory-report': 'inventory_report',
  'ask-ai': 'nav_ask_ai',
  'location-map': 'nav_store_map',
  'slam-map': 'nav_slam',
  'robot-status': 'robot_status',
  restock: 'nav_restock',
  'employee-accounts': 'nav_employee_accounts'
}

watchEffect(() => {
  const lang = currentLang.value
  const routeName = typeof route.name === 'string' ? route.name : ''
  const titleKey = routeTitleKeys[routeName]
  const brand = t('brand')
  const pageTitle = titleKey ? t(titleKey) : brand

  document.documentElement.lang = lang
  document.title = routeName === 'idle' ? pageTitle : `${brand} | ${pageTitle}`
})
</script>
