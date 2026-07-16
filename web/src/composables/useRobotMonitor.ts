import {
  computed,
  onBeforeUnmount,
  onMounted,
  reactive,
  ref,
  shallowRef,
} from 'vue'
import {
  appendBoundedLog,
  buildJointRows,
  mergePluginStatistics,
  normalizeTelemetry,
} from '../model'
import {
  getJointInfo,
  getPluginNames,
  getProcesses,
  getServerVersion,
} from '../services/http'
import { MonitorTransport } from '../services/transport'
import type {
  ConnectionState,
  JointInfo,
  JointTelemetry,
  PluginStatistics,
  ProcessInfo,
  ProcessLogEntry,
  StreamEvent,
} from '../types'

export function useRobotMonitor() {
  const connection = ref<ConnectionState>('connecting')
  const serverVersion = ref('XBot2 server')
  const processes = ref<ProcessInfo[]>([])
  const pluginNames = ref<string[]>([])
  const pluginStats = reactive<Record<string, PluginStatistics>>({})
  const jointInfo = shallowRef<JointInfo>()
  const jointTelemetry = shallowRef<JointTelemetry>()
  const faults = reactive<Record<string, string>>({})
  const logs = reactive<Record<string, ProcessLogEntry[]>>({})
  const errors = reactive<Record<'processes' | 'plugins' | 'joints' | 'stream', string>>({
    processes: '',
    plugins: '',
    joints: '',
    stream: '',
  })
  const loading = reactive({ processes: true, plugins: true, joints: true })

  // Incoming joint messages arrive around 30 Hz. Keep the freshest packet in a
  // plain variable and publish it to Vue at 10 Hz; decoding stays lossless while
  // DOM work remains modest on mobile hardware.
  let pendingJoint: JointTelemetry | undefined
  let lastJointReceivedAt = 0
  let lastServerMessageAt = 0
  let logSequence = 0
  let snapshotTimer: number | undefined
  let freshnessTimer: number | undefined
  const clock = ref(Date.now())
  const retryTimers = new Set<number>()
  const pendingProcessStatuses = new Map<string, string>()

  const transport = new MonitorTransport({
    onConnectionChange: (state) => {
      connection.value = state
    },
    onEvent: handleStreamEvent,
    onProtocolError: (error) => {
      errors.stream = error.message
    },
  })

  function handleStreamEvent(event: StreamEvent): void {
    lastServerMessageAt = Date.now()
    errors.stream = ''

    if (event.kind === 'joint-state') {
      pendingJoint = event.value
      lastJointReceivedAt = Date.now()
      return
    }

    if (event.kind === 'process-output') {
      if (event.out) addLog(event.name, 'stdout', event.out)
      if (event.err) addLog(event.name, 'stderr', event.err)
      return
    }

    const message = event.value
    if (message.type === 'proc_status') {
      const name = String(message.name ?? '')
      const status = String(message.status ?? 'unknown')
      const process = processes.value.find((candidate) => candidate.name === name)
      if (process) process.status = status
      else pendingProcessStatuses.set(name, status)
    } else if (message.type === 'plugin_stats') {
      for (const [name, value] of Object.entries(message)) {
        if (name === 'type' || typeof value !== 'object' || value === null) continue
        const candidate = value as Partial<PluginStatistics>
        pluginStats[name] = mergePluginStatistics(pluginStats[name], candidate)
      }
    } else if (message.type === 'joint_fault') {
      const names = Array.isArray(message.name) ? message.name.map(String) : []
      const codes = Array.isArray(message.fault) ? message.fault.map(String) : []
      names.forEach((name, index) => {
        const code = codes[index] ?? ''
        if (code) faults[name] = code
        else delete faults[name]
      })
    }
  }

  function addLog(name: string, stream: 'stdout' | 'stderr', text: string): void {
    const entry: ProcessLogEntry = {
      id: ++logSequence,
      time: new Date(),
      stream,
      text,
    }
    logs[name] = appendBoundedLog(logs[name] ?? [], entry)
  }

  function clearLogs(name: string): void {
    logs[name] = []
  }

  function scheduleRetry(task: () => Promise<void>, delay = 2_000): void {
    const timer = window.setTimeout(() => {
      retryTimers.delete(timer)
      void task()
    }, delay)
    retryTimers.add(timer)
  }

  async function refreshProcesses(retry = false): Promise<void> {
    loading.processes = true
    try {
      const result = await getProcesses()
      result.forEach((process) => {
        process.status = pendingProcessStatuses.get(process.name) ?? process.status
      })
      processes.value = result
      errors.processes = ''
    } catch (reason) {
      errors.processes = errorMessage(reason)
      if (retry) scheduleRetry(() => refreshProcesses(true))
    } finally {
      loading.processes = false
    }
  }

  async function refreshPlugins(retry = false): Promise<void> {
    loading.plugins = true
    try {
      pluginNames.value = await getPluginNames()
      errors.plugins = ''
    } catch (reason) {
      errors.plugins = errorMessage(reason)
      if (retry) scheduleRetry(() => refreshPlugins(true))
    } finally {
      loading.plugins = false
    }
  }

  async function refreshJoints(retry = false): Promise<void> {
    loading.joints = true
    try {
      const result = await getJointInfo()
      jointInfo.value = result
      if (!jointTelemetry.value) jointTelemetry.value = normalizeTelemetry(result.jstate)
      errors.joints = ''
    } catch (reason) {
      errors.joints = errorMessage(reason)
      if (retry) scheduleRetry(() => refreshJoints(true), 1_000)
    } finally {
      loading.joints = false
    }
  }

  const jointRows = computed(() =>
    buildJointRows(jointInfo.value, jointTelemetry.value, faults),
  )
  const serverFresh = computed(
    () => connection.value === 'connected' && clock.value - lastServerMessageAt < 3_000,
  )
  const robotState = computed<'unavailable' | 'stale' | 'live'>(() => {
    // Read the reactive clock before the early return. On the first render no
    // joint packet has arrived yet; returning before this read would leave the
    // computed value with no reactive dependency, so Vue would cache
    // "unavailable" forever even while telemetry updates the table.
    const now = clock.value
    if (!lastJointReceivedAt) return 'unavailable'
    return now - lastJointReceivedAt < 1_000 ? 'live' : 'stale'
  })
  const runningProcessCount = computed(
    () => processes.value.filter((item) => ['Running', 'Waiting'].includes(item.status)).length,
  )
  const runningPluginCount = computed(
    () => pluginNames.value.filter((name) => pluginStats[name]?.state === 'Running').length,
  )
  const faultCount = computed(() => Object.keys(faults).length)

  onMounted(() => {
    void getServerVersion()
      .then((result) => (serverVersion.value = result.version))
      .catch(() => undefined)
    void refreshProcesses(true)
    void refreshPlugins(true)
    void refreshJoints(true)
    void transport.start()

    snapshotTimer = window.setInterval(() => {
      if (pendingJoint) {
        jointTelemetry.value = pendingJoint
        pendingJoint = undefined
      }
    }, 100)
    freshnessTimer = window.setInterval(() => (clock.value = Date.now()), 250)
  })

  onBeforeUnmount(() => {
    transport.stop()
    if (snapshotTimer !== undefined) window.clearInterval(snapshotTimer)
    if (freshnessTimer !== undefined) window.clearInterval(freshnessTimer)
    retryTimers.forEach((timer) => window.clearTimeout(timer))
  })

  return {
    connection,
    serverVersion,
    serverFresh,
    robotState,
    processes,
    pluginNames,
    pluginStats,
    jointTelemetry,
    jointRows,
    faults,
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
  }
}

function errorMessage(reason: unknown): string {
  return reason instanceof Error ? reason.message : String(reason)
}
