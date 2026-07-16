<script setup lang="ts">
import StatusBadge from './StatusBadge.vue'
import { formatNumber } from '../format'
import type { ConnectionState, JointTelemetry } from '../types'

defineProps<{
  connection: ConnectionState
  serverFresh: boolean
  robotState: 'unavailable' | 'stale' | 'live'
  runningProcesses: number
  processTotal: number
  runningPlugins: number
  pluginTotal: number
  faultCount: number
  jointTotal: number
  telemetry?: JointTelemetry
}>()
</script>

<template>
  <section aria-labelledby="overview-title">
    <div class="section-heading">
      <div>
        <p class="eyebrow">System at a glance</p>
        <h2 id="overview-title">Overview</h2>
      </div>
    </div>

    <div class="metric-grid">
      <article class="metric-card">
        <span class="metric-card__label">Server stream</span>
        <StatusBadge :status="serverFresh ? connection : 'stale'" />
        <strong>{{ serverFresh ? 'Receiving heartbeats' : 'No recent heartbeat' }}</strong>
      </article>
      <article class="metric-card">
        <span class="metric-card__label">Robot telemetry</span>
        <StatusBadge :status="robotState" />
        <strong>{{ jointTotal }} joints discovered</strong>
      </article>
      <article class="metric-card">
        <span class="metric-card__label">Processes</span>
        <strong class="metric-card__number">{{ runningProcesses }}/{{ processTotal }}</strong>
        <span>running or waiting</span>
      </article>
      <article class="metric-card">
        <span class="metric-card__label">Plugins</span>
        <strong class="metric-card__number">{{ runningPlugins }}/{{ pluginTotal }}</strong>
        <span>currently running</span>
      </article>
      <article class="metric-card" :class="{ 'metric-card--alert': faultCount > 0 }">
        <span class="metric-card__label">Joint faults</span>
        <strong class="metric-card__number">{{ faultCount }}</strong>
        <span>{{ faultCount ? 'requires attention' : 'no active reports' }}</span>
      </article>
      <article class="metric-card power-card">
        <span class="metric-card__label">Power</span>
        <div class="power-values">
          <strong>{{ formatNumber(telemetry?.vbatt, 1) }} <small>V</small></strong>
          <strong>{{ formatNumber(telemetry?.ibatt, 1) }} <small>A</small></strong>
        </div>
        <span>latest battery measurements</span>
      </article>
    </div>

    <article class="learning-note">
      <strong>How this page stays current</strong>
      <p>
        Names, limits, and initial states arrive as HTTP snapshots. The changing values arrive
        over one WebSocket as protobuf messages. Each panel can therefore recover independently
        if a ROS service is temporarily unavailable.
      </p>
    </article>
  </section>
</template>
