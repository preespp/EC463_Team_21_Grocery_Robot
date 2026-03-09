<template>
  <div>
    <h2>{{ t('employee_accounts') }}</h2>

    <div class="panel table-panel table-panel-purple">
      <h3>{{ t('existing_employees') }}</h3>
      <div class="table-scroll">
        <table class="table-purple">
          <thead>
            <tr>
              <th>{{ t('employee_id') }}</th>
              <th>{{ t('first_name') }}</th>
              <th>{{ t('last_name') }}</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="emp in employees" :key="emp.employee_ID">
              <td>{{ emp.employee_ID }}</td>
              <td>{{ emp.first_name }}</td>
              <td>{{ emp.last_name }}</td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>

    <div class="panel mt16 table-panel table-panel-teal">
      <h3>{{ t('create_employee') }}</h3>
      <div class="form-row">
        <label class="field grow">
          <span>{{ t('employee_id') }}</span>
          <input v-model="form.employee_ID" placeholder="Ex: 010AAA" />
        </label>
        <label class="field grow">
          <span>{{ t('first_name') }}</span>
          <input v-model="form.first_name" />
        </label>
        <label class="field grow">
          <span>{{ t('last_name') }}</span>
          <input v-model="form.last_name" />
        </label>
      </div>
      <div class="form-row mt16">
        <label class="field grow">
          <span>{{ t('password') }}</span>
          <input v-model="form.password" type="password" />
        </label>
        <label class="field grow">
          <span>{{ t('confirm_password') }}</span>
          <input v-model="form.confirm_password" type="password" />
        </label>
      </div>

      <div class="row mt16">
        <button class="btn" :disabled="busy" @click="createAccount">
          {{ busy ? t('creating') : t('create_account') }}
        </button>
        <div class="muted" v-if="message">{{ message }}</div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, reactive, ref } from 'vue'
import { api } from '../api'
import { t } from '../i18n'

const employees = ref([])
const busy = ref(false)
const message = ref('')
const form = reactive({
  employee_ID: '',
  first_name: '',
  last_name: '',
  password: '',
  confirm_password: ''
})

async function refreshEmployees() {
  const data = await api.employeeAccounts()
  employees.value = data.items || []
}

onMounted(refreshEmployees)

async function createAccount() {
  message.value = ''
  if (!form.employee_ID || !form.first_name || !form.last_name || !form.password || !form.confirm_password) {
    message.value = t('all_fields_required')
    return
  }
  if (form.password !== form.confirm_password) {
    message.value = t('password_mismatch')
    return
  }

  busy.value = true
  try {
    const res = await api.createEmployeeAccount({
      employee_ID: form.employee_ID,
      first_name: form.first_name,
      last_name: form.last_name,
      password: form.password
    })
    message.value = `${t('account_created')}: ${res.employee_ID}`

    form.employee_ID = ''
    form.first_name = ''
    form.last_name = ''
    form.password = ''
    form.confirm_password = ''

    await refreshEmployees()
  } catch (e) {
    message.value = `${t('submit_failed')}: ${e.message}`
  } finally {
    busy.value = false
  }
}
</script>
