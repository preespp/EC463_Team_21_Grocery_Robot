async function request(path, opts = {}) {
  const token = localStorage.getItem('fleet_token')
  const headers = {
    'Content-Type': 'application/json',
    ...(opts.headers || {})
  }
  if (token) headers['Authorization'] = `Bearer ${token}`

  const res = await fetch(path, { ...opts, headers })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`)
  return data
}

export const api = {
  employeeLogin(payload) {
    return request('/api/employee/auth/login', {
      method: 'POST',
      body: JSON.stringify(payload)
    })
  },
  inventoryReport() {
    return request('/api/employee/inventory/report')
  },
  inventoryOptions() {
    return request('/api/employee/inventory/options')
  },
  robotStatus() {
    return request('/api/employee/robot/status')
  },
  employeeAccounts() {
    return request('/api/employee/accounts')
  },
  createEmployeeAccount(payload) {
    return request('/api/employee/accounts/create', {
      method: 'POST',
      body: JSON.stringify(payload)
    })
  },
  submitRestock(payload) {
    return request('/api/employee/restock/submit', {
      method: 'POST',
      body: JSON.stringify(payload)
    })
  }
}
