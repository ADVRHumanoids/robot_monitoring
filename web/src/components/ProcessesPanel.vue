<script setup lang="ts">
import { computed, nextTick, ref, watch } from 'vue'
import type { ProcessInfo, ProcessLogEntry } from '../types'
import StatusBadge from './StatusBadge.vue'

type OutputMode = 'all' | 'selected'
type ConsoleEntry = ProcessLogEntry & { processName: string }

// Keep the console palette aligned with LauncherConsoleItem.qml. The colors
// are deliberately bright enough to remain distinguishable on the dark
// console background without recoloring the process output itself.
const PROCESS_COLORS = [
  '#82AAFF',
  '#89DDFF',
  '#C792EA',
  '#A6E3A1',
  '#9CCFD8',
  '#B4BEFE',
  '#7DCFFF',
  '#A1EFD3',
] as const

const props = defineProps<{
  processes: ProcessInfo[]
  logs: Record<string, ProcessLogEntry[]>
  loading: boolean
  error: string
}>()

const emit = defineEmits<{ refresh: []; clearLogs: [name: string] }>()

const query = ref('')
const selectedName = ref('')
const outputMode = ref<OutputMode>('all')
const followOutput = ref(true)
const consoleElement = ref<HTMLElement>()
const mutedProcessNames = ref<string[]>([])

const filteredProcesses = computed(() => {
  const needle = query.value.trim().toLowerCase()
  if (!needle) return props.processes
  return props.processes.filter((process) =>
    [process.name, process.status, process.category, process.machine, process.cmd, process.docker]
      .join(' ')
      .toLowerCase()
      .includes(needle),
  )
})

const selected = computed(() => {
  const explicit = props.processes.find((process) => process.name === selectedName.value)
  return explicit ?? filteredProcesses.value[0]
})
const selectedVariants = computed(() =>
  selected.value ? Object.values(selected.value.cmdline ?? {}) : [],
)
const runningCount = computed(
  () => props.processes.filter((process) => ['Running', 'Waiting'].includes(process.status)).length,
)
const problemCount = computed(
  () => props.processes.filter((process) => ['Killed', 'Killing'].includes(process.status)).length,
)

/**
 * Per-process buffers are the source of truth. Their IDs come from one global
 * sequence in the monitor composable, so flattening and sorting reconstructs
 * the original cross-process order without storing every line twice.
 */
const aggregateSources = computed(() =>
  [...new Set([...props.processes.map((process) => process.name), ...Object.keys(props.logs)])].sort(),
)
const aggregateLogs = computed<ConsoleEntry[]>(() =>
  Object.entries(props.logs)
    .flatMap(([processName, entries]) =>
      entries.map((entry) => ({ ...entry, processName })),
    )
    .filter((entry) => !mutedProcessNames.value.includes(entry.processName))
    .sort((left, right) => left.id - right.id)
    .slice(-1_000),
)
const selectedLogs = computed<ConsoleEntry[]>(() => {
  if (!selected.value) return []
  return (props.logs[selected.value.name] ?? []).map((entry) => ({
    ...entry,
    processName: selected.value!.name,
  }))
})
const visibleConsoleLogs = computed(() =>
  outputMode.value === 'all' ? aggregateLogs.value : selectedLogs.value,
)

watch(
  () => [
    outputMode.value,
    selected.value?.name,
    visibleConsoleLogs.value[visibleConsoleLogs.value.length - 1]?.id,
  ],
  async () => {
    if (!followOutput.value) return
    await nextTick()
    const element = consoleElement.value
    if (!element) return
    // scrollTop is also supported by lightweight/test DOMs that do not expose
    // the convenience scrollTo() method.
    element.scrollTop = element.scrollHeight
  },
)

function choose(name: string): void {
  selectedName.value = name
}

function clearSelectedLogs(): void {
  if (selected.value) emit('clearLogs', selected.value.name)
}

function setProcessMuted(name: string, muted: boolean): void {
  mutedProcessNames.value = muted
    ? [...mutedProcessNames.value, name]
    : mutedProcessNames.value.filter((candidate) => candidate !== name)
}

function processColor(name: string): string {
  // QML reserves the first color for the launcher's own output, then assigns
  // colors according to the configured process order.
  if (name === 'launcher') return PROCESS_COLORS[0]
  const configuredIndex = props.processes.findIndex((process) => process.name === name)
  if (configuredIndex >= 0) {
    return PROCESS_COLORS[(configuredIndex + 1) % PROCESS_COLORS.length]!
  }

  // Output can arrive before /process/get_list or from an auxiliary source.
  // A small stable hash keeps those sources colored consistently as well.
  const hash = [...name].reduce((value, character) => value + character.charCodeAt(0), 0)
  return PROCESS_COLORS[hash % PROCESS_COLORS.length]!
}
</script>

<template>
  <section aria-labelledby="processes-title">
    <div class="section-heading">
      <div>
        <p class="eyebrow">Launcher inventory</p>
        <h2 id="processes-title">Processes</h2>
      </div>
      <button class="secondary-button" type="button" @click="$emit('refresh')">Refresh list</button>
    </div>

    <div v-if="processes.length" class="process-summary-strip" aria-label="Process summary">
      <div><strong>{{ processes.length }}</strong><span>Total</span></div>
      <div><strong class="summary-good">{{ runningCount }}</strong><span>Active</span></div>
      <div><strong :class="{ 'summary-bad': problemCount }">{{ problemCount }}</strong><span>Problems</span></div>
      <div><strong>{{ aggregateSources.length }}</strong><span>Output sources</span></div>
    </div>

    <label class="search-field process-search">
      <span class="sr-only">Search processes</span>
      <input v-model="query" type="search" placeholder="Search name, state, category, or machine" />
    </label>

    <div v-if="error" class="state-message state-message--error" role="alert">
      <strong>Process list unavailable</strong><span>{{ error }}</span>
    </div>
    <div v-else-if="loading && !processes.length" class="state-message">Loading processes…</div>
    <div v-else-if="!filteredProcesses.length" class="state-message">No matching processes.</div>

    <div v-else class="process-layout">
      <div class="mobile-card-list process-cards">
        <article
          v-for="process in filteredProcesses"
          :key="process.name"
          class="item-card selectable-card"
          :class="{ 'selectable-card--selected': selected?.name === process.name }"
        >
          <button class="process-card-select" type="button" @click="choose(process.name)">
            <span class="item-card__heading">
              <strong>{{ process.name }}</strong>
              <StatusBadge :status="process.status" />
            </span>
            <span class="card-subtitle">{{ process.category }} · {{ process.machine }}</span>
          </button>
          <label class="card-mute-control">
            <input
              type="checkbox"
              :checked="mutedProcessNames.includes(process.name)"
              :aria-label="`Mute ${process.name} in all-process output`"
              @change="setProcessMuted(process.name, ($event.target as HTMLInputElement).checked)"
            />
            <span class="switch-track" aria-hidden="true"><span></span></span>
            <span>Mute in all output</span>
          </label>
        </article>
      </div>

      <div class="desktop-table-wrap process-table">
        <table>
          <thead><tr><th>Name</th><th>Status</th><th>Category</th><th>Machine</th><th>All output</th></tr></thead>
          <tbody>
            <tr
              v-for="process in filteredProcesses"
              :key="process.name"
              :class="{ 'is-selected': selected?.name === process.name }"
            >
              <th scope="row">
                <button class="table-select" type="button" @click="choose(process.name)">{{ process.name }}</button>
              </th>
              <td><StatusBadge :status="process.status" /></td>
              <td>{{ process.category }}</td>
              <td>{{ process.machine }}</td>
              <td>
                <label class="table-mute-control">
                  <input
                    type="checkbox"
                    :checked="mutedProcessNames.includes(process.name)"
                    :aria-label="`Mute ${process.name} in all-process output`"
                    @change="setProcessMuted(process.name, ($event.target as HTMLInputElement).checked)"
                  />
                  <span class="switch-track" aria-hidden="true"><span></span></span>
                  <span>{{ mutedProcessNames.includes(process.name) ? 'Muted' : 'Included' }}</span>
                </label>
              </td>
            </tr>
          </tbody>
        </table>
      </div>

      <aside v-if="selected" class="detail-panel process-detail" :aria-label="`${selected.name} details`">
        <div class="detail-panel__heading">
          <div><p class="eyebrow">Selected process</p><h3>{{ selected.name }}</h3></div>
          <StatusBadge :status="selected.status" />
        </div>
        <dl class="compact-details">
          <div><dt>Category</dt><dd>{{ selected.category }}</dd></div>
          <div><dt>Machine</dt><dd>{{ selected.machine }}</dd></div>
          <div><dt>Visibility</dt><dd>{{ selected.visible ? 'Visible' : 'Hidden' }}</dd></div>
          <div><dt>Variants</dt><dd>{{ selectedVariants.length || 'None' }}</dd></div>
          <div class="detail-wide">
            <dt>Container</dt>
            <dd>{{ selected.docker || 'Host environment' }}</dd>
          </div>
          <div class="detail-wide command-detail">
            <dt>Command</dt>
            <dd><code>{{ selected.cmd || 'Not specified' }}</code></dd>
          </div>
        </dl>
        <details v-if="selectedVariants.length" class="variant-details">
          <summary>Configuration variants</summary>
          <ul>
            <li v-for="variant in selectedVariants" :key="variant.name">
              <strong>{{ variant.name }}</strong>: {{ variant.options.join(', ') }}
            </li>
          </ul>
        </details>
      </aside>
    </div>

    <section class="detail-panel output-workspace" aria-labelledby="process-output-title">
      <div class="output-workspace__heading">
        <div>
          <p class="eyebrow">Read-only console</p>
          <h3 id="process-output-title">Process output</h3>
        </div>
        <div class="segmented-control" aria-label="Output scope">
          <button
            type="button"
            :class="{ active: outputMode === 'all' }"
            :aria-pressed="outputMode === 'all'"
            @click="outputMode = 'all'"
          >All processes</button>
          <button
            type="button"
            :class="{ active: outputMode === 'selected' }"
            :aria-pressed="outputMode === 'selected'"
            :disabled="!selected"
            @click="outputMode = 'selected'"
          >{{ selected ? selected.name : 'Selected' }}</button>
        </div>
      </div>

      <div class="console-toolbar">
        <strong>
          <template v-if="outputMode === 'all'">{{ visibleConsoleLogs.length }} visible entries</template>
          <template v-else>{{ selected?.name }} · {{ visibleConsoleLogs.length }} entries</template>
        </strong>
        <span v-if="outputMode === 'all'" class="muted-summary">
          {{ mutedProcessNames.length }} muted
        </span>
        <label><input v-model="followOutput" type="checkbox" /> Follow</label>
        <button
          v-if="outputMode === 'selected'"
          type="button"
          class="text-button"
          @click="clearSelectedLogs"
        >Clear locally</button>
      </div>
      <div
        ref="consoleElement"
        class="log-console aggregate-console"
        tabindex="0"
        :aria-label="outputMode === 'all' ? 'Aggregated read-only process output' : 'Read-only process output'"
      >
        <p v-if="!visibleConsoleLogs.length" class="log-console__empty">
          {{ outputMode === 'all'
            ? 'No unmuted output received since this page connected.'
            : 'No output received from this process since the page connected.' }}
        </p>
        <div
          v-for="entry in visibleConsoleLogs"
          :key="entry.id"
          :class="`log-line aggregate-log-line log-line--${entry.stream}`"
        >
          <time>{{ entry.time.toLocaleTimeString() }}</time>
          <strong :style="{ color: processColor(entry.processName) }">[{{ entry.processName }}]</strong>
          <span>{{ entry.text }}</span>
        </div>
      </div>
      <p class="console-footnote">
        <template v-if="outputMode === 'all'">
          Muting only filters this browser view; buffered lines reappear when unmuted. The newest
          1,000 visible entries are shown.
        </template>
        <template v-else>Clearing affects only this browser's buffer for {{ selected?.name }}.</template>
      </p>
    </section>
  </section>
</template>
