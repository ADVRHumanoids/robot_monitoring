export type ConnectionState =
  | 'connecting'
  | 'connected'
  | 'reconnecting'
  | 'disconnected'

export interface ProcessInfo {
  name: string
  status: string
  cmdline: Record<string, ProcessVariant>
  cmd?: string
  /** Container name/image; the server calls this field "docker" for QML compatibility. */
  docker?: string
  machine: string
  visible: boolean
  category: string
}

export interface ProcessVariant {
  name: string
  default: number
  help: string
  type: 'combo' | 'check' | string
  options: string[]
}

export interface PluginStatistics {
  run_time: number
  expected_period: number
  state: string
}

export interface DiagnosticValue {
  key: string
  value: string
}

export interface DiagnosticStatusEntry {
  level: number
  name: string
  message: string
  hardwareId: string
  values: DiagnosticValue[]
}

export interface DiagnosticsSnapshot {
  stamp: number
  frameId: string
  status: DiagnosticStatusEntry[]
}

export interface DiagnosticTreeNode {
  label: string
  path: string
  level: number
  status?: DiagnosticStatusEntry
  children: DiagnosticTreeNode[]
}

/** JSON shape returned by GET /joint_states/info. */
export interface JointInfo {
  success: boolean
  message: string
  jnames: string[]
  qmin: number[]
  qmax: number[]
  vmax: number[]
  taumax: number[]
  jstate: Partial<JointTelemetry>
}

/**
 * Names match the protobuf fields used by the existing QML client. Arrays are
 * positional: element i belongs to JointInfo.jnames[i].
 */
export interface JointTelemetry {
  linkPos: number[]
  motPos: number[]
  linkVel: number[]
  motVel: number[]
  tor: number[]
  motorTemp: number[]
  driverTemp: number[]
  posRef: number[]
  velRef: number[]
  torRef: number[]
  k: number[]
  d: number[]
  aux: Record<string, { value: number[] } | number[]>
  vbatt: number
  ibatt: number
  motorStatus: number[]
  brakeStatus: boolean[]
  motTor: number[]
  stamp?: number
}

export interface JointRow {
  index: number
  name: string
  linkPosition?: number
  motorPosition?: number
  referencePosition?: number
  positionError?: number
  motorVelocity?: number
  torque?: number
  referenceTorque?: number
  motorTemperature?: number
  driverTemperature?: number
  stiffness?: number
  damping?: number
  minimum?: number
  maximum?: number
  velocityLimit?: number
  torqueLimit?: number
  fault?: string
}

export interface ProcessLogEntry {
  id: number
  time: Date
  stream: 'stdout' | 'stderr'
  text: string
}

export type JsonStreamMessage = {
  type?: string
  [key: string]: unknown
}

export type StreamEvent =
  | { kind: 'json'; value: JsonStreamMessage }
  | { kind: 'joint-state'; value: JointTelemetry }
  | { kind: 'process-output'; name: string; out: string; err: string }

export const EMPTY_JOINT_TELEMETRY: JointTelemetry = {
  linkPos: [],
  motPos: [],
  linkVel: [],
  motVel: [],
  tor: [],
  motorTemp: [],
  driverTemp: [],
  posRef: [],
  velRef: [],
  torRef: [],
  k: [],
  d: [],
  aux: {},
  vbatt: 0,
  ibatt: 0,
  motorStatus: [],
  brakeStatus: [],
  motTor: [],
}
