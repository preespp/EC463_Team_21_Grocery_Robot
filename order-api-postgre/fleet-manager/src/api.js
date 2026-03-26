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

export async function fetchWithAuth(path, opts = {}) {
  const token = localStorage.getItem('fleet_token')
  const headers = {
    ...(opts.headers || {})
  }
  if (token) headers.Authorization = `Bearer ${token}`
  return fetch(path, { ...opts, headers })
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
  downloadInventoryReport() {
    return fetchWithAuth('/api/employee/inventory/download-report')
      .then(res => res.blob())
      .then(blob => {
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        const timestamp = new Date().toISOString().slice(0, 19).replace(/:/g, '-');
        a.href = url;
        a.download = `inventory_report_${timestamp}.csv`;
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url);
        document.body.removeChild(a);
      });
  },
  semanticMap() {
    return request('/api/maps/semantic/current')
  },
  saveSemanticMap(bundle, options = {}) {
    return request('/api/maps/semantic/current', {
      method: 'PUT',
      body: JSON.stringify({
        bundle,
        change_summary: options.changeSummary || ''
      })
    })
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
