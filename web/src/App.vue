<script setup lang="ts">
import { ref } from 'vue'
import JointsPanel from './components/JointsPanel.vue'
import DiagnosticsPanel from './components/DiagnosticsPanel.vue'
import OverviewPanel from './components/OverviewPanel.vue'
import PluginsPanel from './components/PluginsPanel.vue'
import ProcessesPanel from './components/ProcessesPanel.vue'
import StatusBadge from './components/StatusBadge.vue'
import { useRobotMonitor } from './composables/useRobotMonitor'

type Section = 'overview' | 'processes' | 'plugins' | 'joints' | 'diagnostics'

const sections: { id: Section; label: string; symbol: string }[] = [
  { id: 'overview', label: 'Overview', symbol: '⌂' },
  { id: 'processes', label: 'Processes', symbol: '▣' },
  { id: 'plugins', label: 'Plugins', symbol: '◆' },
  { id: 'joints', label: 'Joints', symbol: '⌁' },
  { id: 'diagnostics', label: 'Diagnostics', symbol: '⚕' },
]
const activeSection = ref<Section>('overview')

const {
  connection,
  serverVersion,
  serverFresh,
  robotState,
  processes,
  pluginNames,
  pluginStats,
  diagnostics,
  diagnosticTree,
  jointTelemetry,
  jointRows,
  logs,
  errors,
  loading,
  runningProcessCount,
  runningPluginCount,
  faultCount,
  clearLogs,
  refreshProcesses,
  refreshPlugins,
  refreshJoints,
} = useRobotMonitor()
</script>

<template>
  <div class="app-shell">
    <header class="top-bar">
      <div>
        <p class="top-bar__product">XBot2</p>
        <h1>Robot Monitor</h1>
      </div>
      <div class="top-bar__status">
        <span class="top-bar__version">{{ serverVersion }}</span>
        <StatusBadge :status="connection" />
      </div>
    </header>

    <aside class="side-navigation" aria-label="Primary navigation">
      <div class="side-navigation__brand"><span aria-hidden="true">XB</span><strong>Monitor</strong></div>
      <nav>
        <button
          v-for="section in sections"
          :key="section.id"
          type="button"
          :class="{ active: activeSection === section.id }"
          :aria-current="activeSection === section.id ? 'page' : undefined"
          @click="activeSection = section.id"
        >
          <span aria-hidden="true">{{ section.symbol }}</span>{{ section.label }}
        </button>
      </nav>
      <div class="side-navigation__health">
        <span>Robot</span><StatusBadge :status="robotState" />
      </div>
    </aside>

    <main id="main-content" class="main-content">
      <OverviewPanel
        v-if="activeSection === 'overview'"
        :connection="connection"
        :server-fresh="serverFresh"
        :robot-state="robotState"
        :running-processes="runningProcessCount"
        :process-total="processes.length"
        :running-plugins="runningPluginCount"
        :plugin-total="pluginNames.length"
        :fault-count="faultCount"
        :joint-total="jointRows.length"
        :telemetry="jointTelemetry"
      />
      <ProcessesPanel
        v-else-if="activeSection === 'processes'"
        :processes="processes"
        :logs="logs"
        :loading="loading.processes"
        :error="errors.processes"
        @refresh="refreshProcesses()"
        @clear-logs="clearLogs"
      />
      <PluginsPanel
        v-else-if="activeSection === 'plugins'"
        :names="pluginNames"
        :stats="pluginStats"
        :loading="loading.plugins"
        :error="errors.plugins"
        @refresh="refreshPlugins()"
      />
      <JointsPanel
        v-else-if="activeSection === 'joints'"
        :rows="jointRows"
        :robot-state="robotState"
        :loading="loading.joints"
        :error="errors.joints"
        @refresh="refreshJoints()"
      />
      <DiagnosticsPanel
        v-else
        :snapshot="diagnostics"
        :tree="diagnosticTree"
      />
    </main>

    <nav class="bottom-navigation" aria-label="Primary navigation">
      <button
        v-for="section in sections"
        :key="section.id"
        type="button"
        :class="{ active: activeSection === section.id }"
        :aria-current="activeSection === section.id ? 'page' : undefined"
        @click="activeSection = section.id"
      >
        <span aria-hidden="true">{{ section.symbol }}</span>{{ section.label }}
      </button>
    </nav>
  </div>
</template>
