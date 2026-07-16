<script setup lang="ts">
import { computed } from 'vue'

const props = defineProps<{ status: string }>()

const tone = computed(() => {
  const status = props.status.toLowerCase()
  if (['connected', 'live', 'running', 'initialized', 'ok'].includes(status)) return 'good'
  if (['waiting', 'starting', 'stopping', 'killing', 'stale', 'reconnecting'].includes(status)) {
    return 'warning'
  }
  if (['killed', 'aborted', 'initfailed', 'fault', 'error'].includes(status)) return 'bad'
  return 'neutral'
})
</script>

<template>
  <span class="status-badge" :class="`status-badge--${tone}`">
    <span class="status-badge__dot" aria-hidden="true"></span>
    {{ status || 'Unknown' }}
  </span>
</template>
