<template>
  <div class="idle-screen" @click="goLogin">
    <div class="cute-bot" aria-label="Friendly robot">
      <div class="antenna"></div>
      <div class="head">
        <div class="eye left"></div>
        <div class="eye right"></div>
        <div class="cheek left"></div>
        <div class="cheek right"></div>
        <div class="mouth"></div>
      </div>
    </div>
    <h1>{{ t('idle_title') }}</h1>
    <p>{{ t('idle_subtitle') }}</p>
  </div>
</template>

<script setup>
import { useRouter } from 'vue-router'
import { t } from '../i18n'

const router = useRouter()
function goLogin() {
  router.push('/login')
}
</script>
