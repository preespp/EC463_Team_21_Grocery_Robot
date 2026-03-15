<template>
  <div>
    <h2>{{ t('inventory_report') }}</h2>
    <div class="cards-grid">
      <div class="metric-card"><div class="label">{{ t('total_products') }}</div><div class="value">{{ report.summary.total_products }}</div></div>
      <div class="metric-card"><div class="label">{{ t('occupied') }}</div><div class="value">{{ report.summary.occupied_products }}</div></div>
      <div class="metric-card"><div class="label">{{ t('empty') }}</div><div class="value">{{ report.summary.empty_products }}</div></div>
      <div class="metric-card"><div class="label">{{ t('units_in_stock') }}</div><div class="value">{{ report.summary.units_in_stock }}</div></div>
      <div class="metric-card"><div class="label">{{ t('low_stock') }}</div><div class="value">{{ report.summary.low_stock_products }}</div></div>
      <div class="metric-card"><div class="label">{{ t('restock_today') }}</div><div class="value">{{ report.summary.restock_requests_today }}</div></div>
    </div>

    <div class="panel mt16 table-panel table-panel-cyan">
      <h3>{{ t('category_summary') }}</h3>
      <table class="table-cyan">
        <thead><tr><th>{{ t('category') }}</th><th>{{ t('products') }}</th><th>{{ t('units') }}</th></tr></thead>
        <tbody>
          <tr v-for="row in report.category_summary" :key="row.category">
            <td>{{ row.category }}</td>
            <td>{{ row.products }}</td>
            <td>{{ row.units }}</td>
          </tr>
        </tbody>
      </table>
    </div>

    <div class="panel mt16 table-panel table-panel-orange">
      <h3>{{ t('inventory_details') }}</h3>
      <div class="table-scroll">
        <table class="table-orange">
          <thead><tr><th>{{ t('id') }}</th><th>{{ t('name') }}</th><th>{{ t('category') }}</th><th>{{ t('stock') }}</th><th>X</th><th>Y</th><th>Z</th></tr></thead>
          <tbody>
            <tr v-for="it in report.items" :key="it.product_id">
              <td>{{ it.product_id }}</td>
              <td>{{ it.product_name }}</td>
              <td>{{ it.category_name }}</td>
              <td>{{ it.stock }}</td>
              <td>{{ it.x }}</td>
              <td>{{ it.y }}</td>
              <td>{{ it.z }}</td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, reactive } from 'vue'
import { api } from '../api'
import { t } from '../i18n'

const report = reactive({
  summary: {
    total_products: 0,
    occupied_products: 0,
    empty_products: 0,
    units_in_stock: 0,
    low_stock_products: 0,
    restock_requests_today: 0
  },
  category_summary: [],
  items: []
})

onMounted(async () => {
  const data = await api.inventoryReport()
  Object.assign(report, data)
})
</script>
