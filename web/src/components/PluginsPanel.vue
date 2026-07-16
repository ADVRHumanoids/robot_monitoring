<script setup lang="ts">
import { computed } from 'vue'
import { formatNumber, formatPercent } from '../format'
import { pluginUtilization } from '../model'
import type { PluginStatistics } from '../types'
import StatusBadge from './StatusBadge.vue'

const props = defineProps<{
  names: string[]
  stats: Record<string, PluginStatistics>
  loading: boolean
  error: string
}>()

defineEmits<{ refresh: [] }>()

const rows = computed(() =>
  props.names.map((name) => ({
    name,
    statistics: props.stats[name],
    utilization: pluginUtilization(props.stats[name]),
    // Do not use a truthiness check here: zero is a real, displayable runtime.
    runtimeMs: props.stats[name] ? props.stats[name].run_time * 1000 : undefined,
    periodMs:
      props.stats[name] && props.stats[name].expected_period > 0
        ? props.stats[name].expected_period * 1000
        : undefined,
  })),
)
</script>

<template>
  <section aria-labelledby="plugins-title">
    <div class="section-heading">
      <div>
        <p class="eyebrow">XBotCore tasks</p>
        <h2 id="plugins-title">Plugins</h2>
      </div>
      <button class="secondary-button" type="button" @click="$emit('refresh')">Refresh list</button>
    </div>

    <div v-if="error" class="state-message state-message--error" role="alert">
      <strong>Plugin list unavailable</strong><span>{{ error }}</span>
    </div>
    <div v-else-if="loading && !names.length" class="state-message">Loading plugins…</div>
    <div v-else-if="!names.length" class="state-message">No plugins were reported.</div>

    <div v-else class="mobile-card-list plugin-cards">
      <article v-for="row in rows" :key="row.name" class="item-card">
        <div class="item-card__heading">
          <strong>{{ row.name }}</strong>
          <StatusBadge :status="row.statistics?.state ?? 'Waiting for telemetry'" />
        </div>
        <dl class="compact-details">
          <div><dt>Runtime</dt><dd>{{ formatNumber(row.runtimeMs, 2) }} ms</dd></div>
          <div><dt>Period</dt><dd>{{ formatNumber(row.periodMs, 2) }} ms</dd></div>
          <div><dt>Utilization</dt><dd>{{ formatPercent(row.utilization) }}</dd></div>
        </dl>
        <progress
          v-if="row.utilization !== undefined"
          :value="Math.min(row.utilization, 100)"
          max="100"
          :aria-label="`${row.name} runtime utilization`"
        ></progress>
      </article>
    </div>

    <div v-if="rows.length" class="desktop-table-wrap">
      <table>
        <thead><tr><th>Name</th><th>State</th><th>Runtime</th><th>Expected period</th><th>Utilization</th></tr></thead>
        <tbody>
          <tr v-for="row in rows" :key="row.name">
            <th scope="row">{{ row.name }}</th>
            <td><StatusBadge :status="row.statistics?.state ?? 'Waiting for telemetry'" /></td>
            <td>{{ formatNumber(row.runtimeMs, 2) }} ms</td>
            <td>{{ formatNumber(row.periodMs, 2) }} ms</td>
            <td>{{ formatPercent(row.utilization) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>
</template>
