import { createRouter, createWebHistory } from 'vue-router'
import IdleView from '../views/IdleView.vue'
import LoginView from '../views/LoginView.vue'
import FleetLayout from '../layouts/FleetLayout.vue'
import InventoryReportView from '../views/InventoryReportView.vue'
import AskAIView from '../views/AskAIView.vue'
import LocationMapView from '../views/LocationMapView.vue'
import SlamMapView from '../views/SlamMapView.vue'
import RobotStatusView from '../views/RobotStatusView.vue'
import RestockView from '../views/RestockView.vue'
import EmployeeAccountsView from '../views/EmployeeAccountsView.vue'

const routes = [
  { path: '/', name: 'idle', component: IdleView },
  { path: '/login', name: 'login', component: LoginView },
  {
    path: '/app',
    component: FleetLayout,
    meta: { requiresAuth: true },
    children: [
      { path: '', redirect: '/app/inventory-report' },
      { path: 'inventory-report', name: 'inventory-report', component: InventoryReportView },
      { path: 'ask-ai', name: 'ask-ai', component: AskAIView },
      { path: 'location-map', name: 'location-map', component: LocationMapView },
      { path: 'slam-map', name: 'slam-map', component: SlamMapView },
      { path: 'robot-status', name: 'robot-status', component: RobotStatusView },
      { path: 'restock', name: 'restock', component: RestockView },
      { path: 'employee-accounts', name: 'employee-accounts', component: EmployeeAccountsView }
    ]
  }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

router.beforeEach((to) => {
  if (!to.meta.requiresAuth) return true
  const token = localStorage.getItem('fleet_token')
  if (!token) return '/login'
  return true
})

export default router
