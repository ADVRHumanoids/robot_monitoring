import type { JointInfo, ProcessInfo } from '../types'

export class ApiError extends Error {
  constructor(
    message: string,
    public readonly status?: number,
  ) {
    super(message)
    this.name = 'ApiError'
  }
}

/**
 * All requests are same-origin. During development Vite proxies these paths to
 * aiohttp; in production the Vue files and API share the XBot2 server origin.
 */
async function getJson<T>(path: string): Promise<T> {
  const response = await fetch(path, { headers: { Accept: 'application/json' } })
  if (!response.ok) {
    throw new ApiError(`${path} returned HTTP ${response.status}`, response.status)
  }

  const value = (await response.json()) as T & { success?: boolean; message?: string }
  if (value && value.success === false) {
    throw new ApiError(value.message || `${path} reported a failure`)
  }
  return value
}

export function getServerVersion(): Promise<{ version: string }> {
  return getJson('/version')
}

export function getProcesses(): Promise<ProcessInfo[]> {
  return getJson('/process/get_list')
}

export async function getPluginNames(): Promise<string[]> {
  const response = await getJson<{ plugins: string[] }>('/plugin/get_list')
  return response.plugins
}

export function getJointInfo(): Promise<JointInfo> {
  return getJson('/joint_states/info')
}
