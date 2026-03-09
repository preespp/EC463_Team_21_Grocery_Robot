<template>
  <div>
    <h2>{{ t('restock_page') }}</h2>
    <div class="panel">
      <div class="form-row">
        <label class="field grow">
          <span>{{ t('product') }}</span>
          <select v-model="selectedProductId">
            <option disabled value="">{{ t('select_product') }}</option>
            <option v-for="it in options" :key="it.product_id" :value="String(it.product_id)">
              {{ it.product_name }} ({{ t('stock').toLowerCase() }}: {{ it.stock }})
            </option>
          </select>
        </label>
        <label class="field qty-field">
          <span>{{ t('quantity') }}</span>
          <input v-model.number="qty" type="number" min="1" />
        </label>
        <button class="btn" @click="addItem">{{ t('add') }}</button>
      </div>

      <table v-if="items.length > 0" class="mt16 table-green">
        <thead><tr><th>{{ t('product') }}</th><th>{{ t('quantity') }}</th><th></th></tr></thead>
        <tbody>
          <tr v-for="(it, idx) in items" :key="`${it.product_id}-${idx}`">
            <td>{{ it.name }}</td>
            <td>{{ it.qty }}</td>
            <td><button class="btn ghost small" @click="removeItem(idx)">{{ t('remove') }}</button></td>
          </tr>
        </tbody>
      </table>

      <div class="row mt16">
        <button class="btn" :disabled="items.length===0 || busy" @click="submitRestock">
          {{ busy ? t('submitting') : t('submit_restock') }}
        </button>
        <div class="muted" v-if="message">{{ message }}</div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, ref } from 'vue'
import { api } from '../api'
import { t } from '../i18n'

const options = ref([])
const selectedProductId = ref('')
const qty = ref(1)
const items = ref([])
const message = ref('')
const busy = ref(false)

onMounted(async () => {
  const data = await api.inventoryOptions()
  options.value = data.items || []
})

function addItem() {
  message.value = ''
  const option = options.value.find((it) => String(it.product_id) === selectedProductId.value)
  if (!option) {
    message.value = t('select_product_first')
    return
  }
  const q = Number(qty.value)
  if (!Number.isFinite(q) || q <= 0) {
    message.value = t('qty_gt_zero')
    return
  }

  const existing = items.value.find((it) => it.product_id === option.product_id)
  if (existing) {
    existing.qty += q
  } else {
    items.value.push({
      product_id: option.product_id,
      name: option.product_name,
      qty: q
    })
  }
}

function removeItem(index) {
  items.value.splice(index, 1)
}

async function submitRestock() {
  busy.value = true
  message.value = ''
  try {
    const payload = {
      items: items.value.map((it) => ({ product_id: it.product_id, qty: it.qty }))
    }
    const res = await api.submitRestock(payload)
    message.value = `${t('restock_submitted')}: ${res.restock_ID}`
    items.value = []
  } catch (e) {
    message.value = `${t('submit_failed')}: ${e.message}`
  } finally {
    busy.value = false
  }
}
</script>
