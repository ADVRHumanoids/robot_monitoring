# XBot2 Web Monitor: a guided tour

This directory contains a deliberately small, read-only Vue application. It monitors processes,
XBotCore plugins, and joint telemetry using the APIs that already exist in
`server/src/xbot2_gui_server`. It contains no process, plugin, joint, or server command calls.

The application is designed to be read as well as run. Start with `src/App.vue`, then follow the
data down into `src/composables/useRobotMonitor.ts`, `src/services/transport.ts`, and finally the
presentation components.

## Why Vue if I already know QML?

A Vue single-file component (`.vue`) has three familiar parts:

```vue
<script setup lang="ts">
// Properties, computed values, and behavior: similar to a QML component's logic.
</script>

<template>
  <!-- The declarative object tree: similar in spirit to QML's visual tree. -->
</template>
```

CSS is kept globally in `src/style.css` because this application has one visual system. Vue can
also scope CSS to a component, but a single token-based stylesheet makes the mobile/desktop
breakpoints easier to study.

Some useful QML-to-Vue correspondences are:

| QML idea | Vue equivalent |
| --- | --- |
| `property` | `ref`, `reactive`, or a component prop |
| readonly/bound property | `computed` |
| `signal` | an emitted component event |
| signal handler | `@event="handler"` |
| `Repeater` | `v-for` |
| `visible` | `v-if` or `v-show` |
| responsive layouts | normal CSS Grid/Flexbox media queries |

TypeScript adds types to JavaScript. Here the types are especially valuable because joint names
and live values travel in different messages. The compiler catches many accidental mismatches
before the browser runs the code.

## Data flow: snapshots plus streams

```text
GET /process/get_list ───────┐
GET /plugin/get_list ────────┼─> useRobotMonitor ─> Vue components
GET /joint_states/info ──────┤         ▲
                             │         │ decoded events
GET /ws ─> MonitorTransport ─┘         │
              │                        │
              └─ protobuf Message ─────┘
```

HTTP is used for snapshots: names, categories, machines, joint limits, and initial states. The
WebSocket carries values that change continuously. Each HTTP request has its own loading, error,
and retry state, so an unavailable ROS plugin service does not hide process or joint information.

The server's high-rate streams historically use UDP. Browser JavaScript cannot create a UDP
socket, so after opening `/ws` the client sends:

```json
{ "type": "request_ws_udp_tunnel" }
```

This is only a transport subscription. The server mirrors its UDP protobuf messages to that
WebSocket while continuing to send UDP to native QML clients. No robot command is involved.

## The protobuf envelope

`proto/generic.proto` defines a `Message` that can contain JSON text, joint telemetry, process
output, video, or point-cloud data. Vite copies the repository's `.proto` files beside the built
application. At startup, `protobufjs` loads `generic.proto` and its imports. The web monitor routes
only the three branches it needs and safely ignores future branches.

The server also sends heartbeat frames as plain WebSocket text. Therefore
`services/transport.ts` first distinguishes text from binary, then decodes the binary envelope.
A malformed individual frame is reported but does not close the connection.

## Why joint arrays need special care

To save bandwidth, a streamed joint-state protobuf contains arrays but no names. The HTTP response
from `/joint_states/info` supplies the stable ordering:

```text
jnames[i]  <-> linkPos[i] <-> motPos[i] <-> tor[i] <-> qmin[i] ...
```

`buildJointRows()` in `src/model.ts` is the single boundary that performs this positional join.
Missing array elements become `undefined` and are displayed as an em dash; they never shift the
remaining joints. Only display formatting rounds numbers—the underlying values remain unchanged.

Joint packets may arrive around 30 Hz. The transport decodes all of them, but the composable
publishes the freshest snapshot to Vue every 100 ms. This avoids rebuilding a large table 30 times
per second on a phone while still showing current data.

## Reactivity and ownership

- `services/transport.ts` owns WebSocket/protobuf details and reconnects with exponential backoff.
- `services/http.ts` owns the four read-only HTTP calls.
- `composables/useRobotMonitor.ts` owns application state and translates stream events into it.
- `model.ts` contains pure transformations that are easy to unit-test.
- Components receive typed props and emit local UI intentions such as refresh or clear-log.

Process output is retained only in browser memory. Each process has a 200-entry cap so an active
console cannot grow memory forever. The Processes page also reconstructs a chronological aggregate
from the per-process buffers using their global sequence IDs. Per-process mute switches in the
main table (and mobile cards) filter only that aggregate view: no messages are sent and muted
lines remain available for unmuting.
“Clear locally” does not call the server.

## Responsive design

The base CSS targets narrow screens first. At 768 px the bottom navigation becomes a left rail and
cards use multiple columns. At 1200 px process and joint cards become dense tables with sticky
detail panels. Desktop-only content is still present in the document but selected with media
queries—there is no JavaScript screen-size logic to keep synchronized.

Colors come from CSS custom properties, follow the system light/dark preference, and are never the
only status indicator. Controls have at least a 44 px target, keyboard focus is visible, and the
reduced-motion preference is respected.

## Development and production

Requirements: a current Node.js installation and the Python server on port 8080.

```bash
cd web
npm install
npm run dev
```

Open the Vite URL ending in `/monitor/`. Vite proxies the HTTP and WebSocket paths to
`http://localhost:8080`, so frontend code always uses production-like same-origin URLs.

Useful checks are:

```bash
npm run typecheck
npm test
npm run build:server
```

The production command writes to `server/src/xbot2_gui_server/monitor/`. Those generated files are
included as Python package data, which means the deployed robot does not need Node.js. Start the
normal `xbot2_gui_server`; `/` redirects to `/monitor/`, while the legacy QML/WebAssembly page stays
available at `/webui/xbot2_gui.html`.

From the repository root, the same production build is available as a convenient wrapper:

```bash
./regenerate_web.bash
```

It uses the existing `node_modules` directory when present, or performs a reproducible `npm ci`
from `package-lock.json` when dependencies are missing. Pass `--clean-install` to force that clean
dependency installation before rebuilding.

## Safe extension points

To add another read-only stream, first add its wire type in `types.ts`, decode it into a
`StreamEvent` in the transport, update state in the composable, and only then render it. Keeping
that direction prevents a component from becoming coupled to protobuf details. Any future command
feature should use a separate, visibly named service and explicit confirmation UX rather than
quietly extending the monitoring transport.
