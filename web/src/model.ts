import type {
  DiagnosticStatusEntry,
  DiagnosticsSnapshot,
  DiagnosticTreeNode,
  JointInfo,
  JointRow,
  JointTelemetry,
  PluginStatistics,
  ProcessLogEntry,
} from './types'
import { EMPTY_JOINT_TELEMETRY } from './types'

const numericArray = (value: unknown): number[] =>
  Array.isArray(value) ? value.map(Number) : []

/** Convert partial HTTP/protobuf shapes into one predictable UI shape. */
export function normalizeTelemetry(value: Partial<JointTelemetry>): JointTelemetry {
  return {
    ...EMPTY_JOINT_TELEMETRY,
    ...value,
    linkPos: numericArray(value.linkPos),
    motPos: numericArray(value.motPos),
    linkVel: numericArray(value.linkVel),
    motVel: numericArray(value.motVel),
    tor: numericArray(value.tor),
    motorTemp: numericArray(value.motorTemp),
    driverTemp: numericArray(value.driverTemp),
    posRef: numericArray(value.posRef),
    velRef: numericArray(value.velRef),
    torRef: numericArray(value.torRef),
    k: numericArray(value.k),
    d: numericArray(value.d),
    motorStatus: numericArray(value.motorStatus),
    brakeStatus: Array.isArray(value.brakeStatus) ? value.brakeStatus.map(Boolean) : [],
    motTor: numericArray(value.motTor),
    aux: value.aux ?? {},
    vbatt: Number(value.vbatt ?? 0),
    ibatt: Number(value.ibatt ?? 0),
  }
}

function at(values: number[], index: number): number | undefined {
  const value = values[index]
  return value === undefined || !Number.isFinite(value) ? undefined : value
}

/**
 * Join positional telemetry with the names and limits obtained over HTTP.
 * This is the most important data-model boundary in the application: the
 * streamed protobuf intentionally omits names to save bandwidth.
 */
export function buildJointRows(
  info: JointInfo | undefined,
  telemetry: JointTelemetry | undefined,
  faults: Readonly<Record<string, string>>,
): JointRow[] {
  if (!info) return []
  const state = telemetry ?? normalizeTelemetry(info.jstate)

  return info.jnames.map((name, index) => {
    const linkPosition = at(state.linkPos, index)
    const referencePosition = at(state.posRef, index)
    return {
      index,
      name,
      linkPosition,
      motorPosition: at(state.motPos, index),
      referencePosition,
      positionError:
        linkPosition !== undefined && referencePosition !== undefined
          ? referencePosition - linkPosition
          : undefined,
      motorVelocity: at(state.motVel, index),
      torque: at(state.tor, index),
      referenceTorque: at(state.torRef, index),
      motorTemperature: at(state.motorTemp, index),
      driverTemperature: at(state.driverTemp, index),
      stiffness: at(state.k, index),
      damping: at(state.d, index),
      minimum: at(info.qmin, index),
      maximum: at(info.qmax, index),
      velocityLimit: at(info.vmax, index),
      torqueLimit: at(info.taumax, index),
      fault: faults[name] || undefined,
    }
  })
}

export function appendBoundedLog(
  entries: ProcessLogEntry[],
  entry: ProcessLogEntry,
  limit = 200,
): ProcessLogEntry[] {
  const next = [...entries, entry]
  return next.length > limit ? next.slice(next.length - limit) : next
}

export function pluginUtilization(stats?: PluginStatistics): number | undefined {
  if (!stats || stats.expected_period <= 0) return undefined
  return (stats.run_time / stats.expected_period) * 100
}

/**
 * Merge a plugin sample without throwing away useful timing metadata.
 *
 * A runtime of zero is a valid measurement and must not be treated as absent.
 * The expected period, on the other hand, is effectively task configuration;
 * some Statistics2 samples transiently report it as zero. Once a positive
 * period has been observed, retain it until another positive value arrives.
 */
export function mergePluginStatistics(
  previous: PluginStatistics | undefined,
  sample: Partial<PluginStatistics>,
): PluginStatistics {
  const runtime = Number(sample.run_time)
  const period = Number(sample.expected_period)

  return {
    run_time: Number.isFinite(runtime) ? runtime : (previous?.run_time ?? 0),
    expected_period:
      Number.isFinite(period) && period > 0 ? period : (previous?.expected_period ?? 0),
    state: sample.state === undefined ? (previous?.state ?? 'unknown') : String(sample.state),
  }
}

/** Normalize the loosely typed JSON diagnostics event at the stream boundary. */
export function normalizeDiagnostics(value: Record<string, unknown>): DiagnosticsSnapshot {
  const rawStatuses = Array.isArray(value.status) ? value.status : []

  return {
    stamp: Number(value.stamp ?? 0),
    frameId: String(value.frame_id ?? ''),
    status: rawStatuses
      .filter((status): status is Record<string, unknown> =>
        typeof status === 'object' && status !== null,
      )
      .map((status) => ({
        level: normalizeDiagnosticLevel(status.level),
        name: String(status.name ?? ''),
        message: String(status.message ?? ''),
        hardwareId: String(status.hardware_id ?? ''),
        values: (Array.isArray(status.values) ? status.values : [])
          .filter((item): item is Record<string, unknown> =>
            typeof item === 'object' && item !== null,
          )
          .map((item) => ({
            key: String(item.key ?? ''),
            value: String(item.value ?? ''),
          })),
      })),
  }
}

function normalizeDiagnosticLevel(value: unknown): number {
  const level = Number(value)
  return Number.isFinite(level) ? Math.max(0, Math.min(3, Math.trunc(level))) : 3
}

type MutableDiagnosticNode = Omit<DiagnosticTreeNode, 'children'> & {
  childMap: Map<string, MutableDiagnosticNode>
}

/** Build a hierarchy from slash-separated DiagnosticStatus names. */
export function buildDiagnosticTree(statuses: DiagnosticStatusEntry[]): DiagnosticTreeNode[] {
  const roots = new Map<string, MutableDiagnosticNode>()

  statuses.forEach((status) => {
    const segments = status.name.split('/').filter(Boolean)
    if (!segments.length) segments.push(status.name || '(unnamed)')

    let siblings = roots
    let path = ''
    let node: MutableDiagnosticNode | undefined
    segments.forEach((segment) => {
      path = `${path}/${segment}`
      node = siblings.get(segment)
      if (!node) {
        node = { label: segment, path, level: 0, childMap: new Map() }
        siblings.set(segment, node)
      }
      siblings = node.childMap
    })
    if (node) node.status = status
  })

  const finalize = (node: MutableDiagnosticNode): DiagnosticTreeNode => {
    const children = [...node.childMap.values()]
      .map(finalize)
      .sort((left, right) => left.label.localeCompare(right.label))
    const level = Math.max(node.status?.level ?? 0, ...children.map((child) => child.level))
    return {
      label: node.label,
      path: node.path,
      level,
      status: node.status,
      children,
    }
  }

  return [...roots.values()].map(finalize).sort((left, right) => left.label.localeCompare(right.label))
}
