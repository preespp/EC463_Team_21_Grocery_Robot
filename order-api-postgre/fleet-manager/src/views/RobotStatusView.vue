<template>
  <div>
    <h2>{{ t('robot_status') }}</h2>
    <div class="cards-grid">
      <div v-for="rb in robots" :key="rb.robot_number" class="metric-card robot-card">
        <div class="label">{{ rb.robot_number }}</div>
        <div class="robot-status" :class="rb.status_key">{{ rb.status_text }}</div>
        <div class="muted tiny">{{ t('updated') }} {{ rb.updated_at }}</div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, ref } from 'vue'
import { api } from '../api'
import { t } from '../i18n'

const robots = ref([])

onMounted(async () => {
  const data = await api.robotStatus()
  robots.value = data.robots || []
})
</script>
