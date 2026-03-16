<template>
  <div class="ai-container">
    <h2>{{ t('ask_ai') }}</h2>

    <div class="chat-area" ref="chatArea">
      <div
        v-for="(m,i) in messages"
        :key="i"
        :class="['chat-row', m.role]"
      >
        <div class="bubble">
          {{ m.text }}
        </div>
      </div>
    </div>

    <div class="chat-input">
      <input
        v-model="question"
        @keyup.enter="send"
        placeholder="Ask AI..."
      />
      <button @click="send">{{ t('send') }}</button>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { t } from '../i18n'

const messages = ref([])
const question = ref("")

async function send() {
  if (!question.value.trim()) return

  const q = question.value
  messages.value.push({ role: 'user', text: q })
  question.value = ""

  const res = await fetch("/api/employee/ai/ask", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "Authorization": `Bearer ${localStorage.getItem('fleet_token')}`
    },
    body: JSON.stringify({ question: q })
  })

  const data = await res.json()

  messages.value.push({
    role: 'assistant',
    text: data.answer
  })
}
</script>
