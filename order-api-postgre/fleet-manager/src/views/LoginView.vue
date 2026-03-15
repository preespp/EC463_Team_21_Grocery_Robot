<template>
  <div class="login-page">
    <div class="login-card">
      <div class="row" style="justify-content:flex-end">
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
      </div>
      <h2>{{ t('login_title') }}</h2>
      <p class="muted">{{ t('login_subtitle') }}</p>

      <label class="field">
        <span>{{ t('employee_id') }}</span>
        <input v-model="employee_ID" placeholder="Ex: 001AAA" @keyup.enter="submit" />
      </label>

      <label class="field">
        <span>{{ t('password') }}</span>
        <input v-model="password" type="password" :placeholder="t('password')" @keyup.enter="submit" />
      </label>

      <button class="btn" :disabled="busy" @click="submit">
        {{ busy ? t('signing_in') : t('login') }}
      </button>
      <div class="error" v-if="error">{{ error }}</div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { useRouter } from 'vue-router'
import { api } from '../api'
import { currentLang, setLang, t } from '../i18n'

const router = useRouter()
const employee_ID = ref('')
const password = ref('')
const busy = ref(false)
const error = ref('')

async function submit() {
  error.value = ''
  if (!employee_ID.value.trim() || !password.value.trim()) {
    error.value = t('login_required')
    return
  }

  busy.value = true
  try {
    const data = await api.employeeLogin({
      employee_ID: employee_ID.value.trim(),
      password: password.value
    })
    localStorage.setItem('fleet_token', data.token)
    localStorage.setItem('fleet_employee_id', data.employee.employee_ID)
    localStorage.setItem('fleet_first_name', data.employee.first_name || '')
    localStorage.setItem('fleet_last_name', data.employee.last_name || '')
    router.push('/app/inventory-report')
  } catch (e) {
    error.value = e.message
  } finally {
    busy.value = false
  }
}
</script>
