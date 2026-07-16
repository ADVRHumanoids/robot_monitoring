<script setup lang="ts">
import { computed, ref } from 'vue'
import { formatNumber } from '../format'
import type { JointRow } from '../types'
import StatusBadge from './StatusBadge.vue'

const props = defineProps<{
  rows: JointRow[]
  robotState: 'unavailable' | 'stale' | 'live'
  loading: boolean
  error: string
}>()

defineEmits<{ refresh: [] }>()

const query = ref('')
const selectedName = ref('')
const filteredRows = computed(() => {
  const needle = query.value.trim().toLowerCase()
  return needle ? props.rows.filter((row) => row.name.toLowerCase().includes(needle)) : props.rows
})
const selected = computed(
  () => props.rows.find((row) => row.name === selectedName.value) ?? filteredRows.value[0],
)
const maxTemperature = (row: JointRow): number | undefined => {
  const values = [row.motorTemperature, row.driverTemperature].filter(
    (value): value is number => value !== undefined,
  )
  return values.length ? Math.max(...values) : undefined
}
</script>

<template>
  <section aria-labelledby="joints-title">
    <div class="section-heading">
      <div>
        <p class="eyebrow">Live indexed telemetry</p>
        <h2 id="joints-title">Joints</h2>
      </div>
      <div class="heading-actions">
        <StatusBadge :status="robotState" />
        <button class="secondary-button" type="button" @click="$emit('refresh')">Refresh metadata</button>
      </div>
    </div>

    <label class="search-field">
      <span class="sr-only">Search joints</span>
      <input v-model="query" type="search" placeholder="Search joint name" />
    </label>

    <div v-if="error" class="state-message state-message--error" role="alert">
      <strong>Joint metadata unavailable</strong><span>{{ error }}</span>
    </div>
    <div v-else-if="loading && !rows.length" class="state-message">Waiting for joint metadata…</div>
    <div v-else-if="!filteredRows.length" class="state-message">No matching joints.</div>

    <div v-else class="joint-layout">
      <div class="mobile-card-list joint-cards">
        <button
          v-for="row in filteredRows"
          :key="row.name"
          type="button"
          class="item-card selectable-card"
          :class="{ 'selectable-card--selected': selected?.name === row.name }"
          @click="selectedName = row.name"
        >
          <span class="item-card__heading">
            <strong>{{ row.name }}</strong>
            <StatusBadge :status="row.fault ? 'Fault' : 'OK'" />
          </span>
          <span class="joint-card-values">
            <span><small>Position</small>{{ formatNumber(row.linkPosition) }} rad</span>
            <span><small>Error</small>{{ formatNumber(row.positionError) }} rad</span>
            <span><small>Torque</small>{{ formatNumber(row.torque) }} Nm</span>
            <span><small>Temperature</small>{{ formatNumber(maxTemperature(row), 1) }} °C</span>
          </span>
          <span v-if="row.fault" class="fault-text">{{ row.fault }}</span>
        </button>
      </div>

      <div class="desktop-table-wrap joint-table">
        <table>
          <thead>
            <tr><th>Joint</th><th>Link pos.</th><th>Motor pos.</th><th>Reference</th><th>Error</th><th>Velocity</th><th>Torque</th><th>Torque ref.</th><th>Motor °C</th><th>Driver °C</th><th>K</th><th>D</th></tr>
          </thead>
          <tbody>
            <tr
              v-for="row in filteredRows"
              :key="row.name"
              :class="{ 'is-selected': selected?.name === row.name, 'has-fault': row.fault }"
            >
              <th scope="row"><button class="table-select" type="button" @click="selectedName = row.name">{{ row.name }}</button></th>
              <td>{{ formatNumber(row.linkPosition) }}</td><td>{{ formatNumber(row.motorPosition) }}</td>
              <td>{{ formatNumber(row.referencePosition) }}</td><td>{{ formatNumber(row.positionError) }}</td>
              <td>{{ formatNumber(row.motorVelocity) }}</td><td>{{ formatNumber(row.torque) }}</td>
              <td>{{ formatNumber(row.referenceTorque) }}</td><td>{{ formatNumber(row.motorTemperature, 1) }}</td>
              <td>{{ formatNumber(row.driverTemperature, 1) }}</td><td>{{ formatNumber(row.stiffness, 1) }}</td>
              <td>{{ formatNumber(row.damping, 1) }}</td>
            </tr>
          </tbody>
        </table>
      </div>

      <aside v-if="selected" class="detail-panel joint-detail" :aria-label="`${selected.name} details`">
        <div class="detail-panel__heading">
          <div><p class="eyebrow">Selected joint</p><h3>{{ selected.name }}</h3></div>
          <StatusBadge :status="selected.fault ? 'Fault' : 'OK'" />
        </div>
        <p v-if="selected.fault" class="fault-banner"><strong>Reported fault:</strong> {{ selected.fault }}</p>
        <dl class="telemetry-details">
          <div><dt>Link position</dt><dd>{{ formatNumber(selected.linkPosition) }} rad</dd></div>
          <div><dt>Motor position</dt><dd>{{ formatNumber(selected.motorPosition) }} rad</dd></div>
          <div><dt>Position reference</dt><dd>{{ formatNumber(selected.referencePosition) }} rad</dd></div>
          <div><dt>Position error</dt><dd>{{ formatNumber(selected.positionError) }} rad</dd></div>
          <div><dt>Motor velocity</dt><dd>{{ formatNumber(selected.motorVelocity) }} rad/s</dd></div>
          <div><dt>Measured torque</dt><dd>{{ formatNumber(selected.torque) }} Nm</dd></div>
          <div><dt>Torque reference</dt><dd>{{ formatNumber(selected.referenceTorque) }} Nm</dd></div>
          <div><dt>Motor / driver temp.</dt><dd>{{ formatNumber(selected.motorTemperature, 1) }} / {{ formatNumber(selected.driverTemperature, 1) }} °C</dd></div>
          <div><dt>Position limits</dt><dd>{{ formatNumber(selected.minimum) }} … {{ formatNumber(selected.maximum) }} rad</dd></div>
          <div><dt>Velocity limit</dt><dd>{{ formatNumber(selected.velocityLimit) }} rad/s</dd></div>
          <div><dt>Torque limit</dt><dd>{{ formatNumber(selected.torqueLimit) }} Nm</dd></div>
          <div><dt>Stiffness / damping</dt><dd>{{ formatNumber(selected.stiffness, 1) }} / {{ formatNumber(selected.damping, 1) }}</dd></div>
        </dl>
      </aside>
    </div>
  </section>
</template>
