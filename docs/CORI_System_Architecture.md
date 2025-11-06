# CORI System Architecture

**Complete Technical Documentation: ROS2 & Networking Flow**

---

**Author:** Johnathan Uptegraph
**Last Updated:** November 6, 2025
**System:** CORI (Character-Oriented Robot Interface)
**ROS2 Version:** Humble Hawksbill

---

## Table of Contents

- [System Overview](#system-overview)
- [Architecture Diagram](#architecture-diagram)
- [ROS2 Layer](#ros2-layer)
- [Network Layer](#network-layer)
- [Web Interface Layer](#web-interface-layer)
- [Complete Data Flow](#complete-data-flow)
- [Remote Access Configuration](#remote-access-configuration)
- [Security Architecture](#security-architecture)
- [Troubleshooting Guide](#troubleshooting-guide)
- [Quick Reference](#quick-reference)

---

## System Overview

CORI is a web-controlled robotic system that enables users to control a simulated or physical robot from anywhere in the world using just a web browser. The system seamlessly integrates three critical domains:

### Core Technologies

| Domain | Technology | Purpose |
|--------|------------|---------|
| **Robotics** | ROS2 (Robot Operating System 2) | Real-time robot control and simulation |
| **Frontend** | HTML5, CSS3, JavaScript | Modern, responsive user interface |
| **Backend** | Python, Node.js, Nginx | WebSocket server and reverse proxy |
| **Network** | TCP/IP, WebSocket, HTTP | Secure remote access over internet |

### Key Features

- **Real-time Control** - Low-latency WebSocket communication (<100ms local, <300ms remote)
- **Mobile-Friendly** - Responsive design optimized for phones and tablets
- **Secure Access** - HTTP Basic Authentication with optional HTTPS/TLS
- **Global Reach** - Access from anywhere with internet connection
- **Simplified Interface** - Horizontal head control with intuitive touch controls
- **Interactive Commands** - Color detection and Gazebo environment interaction

---

## Architecture Diagram

### System-Level Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                    INTERNET / CELLULAR DATA                          │
│                                                                        │
│                       User's Phone/Laptop                             │
│                   ┌────────────────────────┐                          │
│                   │   Web Browser          │                          │
│                   │  ┌──────────────────┐  │                          │
│                   │  │ CORI Web UI      │  │                          │
│                   │  │ - Sliders        │  │                          │
│                   │  │ - Buttons        │  │                          │
│                   │  │ - WebSocket      │  │                          │
│                   │  └──────────────────┘  │                          │
│                   └────────────────────────┘                          │
│                              │                                         │
│                              │ HTTP/WebSocket                          │
│                              │ http://74.134.87.123                    │
│                              │ ws://74.134.87.123/ws                   │
└──────────────────────────────┼──────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                   HOME NETWORK ROUTER                                │
│                   Public IP: 74.134.87.123                           │
│                                                                        │
│   Port Forwarding:                                                    │
│   • External 80/8080 → Internal 192.168.1.26:80/8080               │
│                                                                        │
└──────────────────────────────┼──────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│              CORI MACHINE (192.168.1.26)                            │
│                                                                        │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                  NGINX REVERSE PROXY                          │   │
│  │                  Ports: 80, 8080                              │   │
│  │                                                                │   │
│  │  Authentication: HTTP Basic Auth (CORI/connect)               │   │
│  │                                                                │   │
│  │  Routes:                                                       │   │
│  │  • /      → 127.0.0.1:8091  (Web UI Server)                  │   │
│  │  • /api/* → 127.0.0.1:8000  (FastAPI)                        │   │
│  │  • /ws    → 127.0.0.1:8767  (WebSocket Server)               │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                               │                                       │
│         ┌─────────────────────┼────────────────────┐                 │
│         │                     │                    │                 │
│         ▼                     ▼                    ▼                 │
│  ┌────────────┐      ┌────────────┐      ┌──────────────────┐      │
│  │  Web UI    │      │  FastAPI   │      │  WebSocket Node  │      │
│  │  Server    │      │  Server    │      │  (Python/ROS2)   │      │
│  │  :8091     │      │  :8000     │      │  :8767           │      │
│  └────────────┘      └────────────┘      └──────────────────┘      │
│                                                    │                  │
│                                                    │ Publishes        │
│                                                    ▼                  │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              ROS2 MIDDLEWARE (DDS)                            │   │
│  │              Topic-Based Publish/Subscribe                    │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                    │                  │
│                                                    ▼                  │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              ROS2 TOPICS                                      │   │
│  │              /model/cori/joint/head_joint/cmd_pos            │   │
│  │              Type: std_msgs/Float64                           │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                    │                  │
│                                                    │ Subscribes       │
│                                                    ▼                  │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              GAZEBO SIMULATION                                │   │
│  │              - Physics Engine                                 │   │
│  │              - CORI Robot Model                               │   │
│  │              - Joint Controllers                               │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                        │
└──────────────────────────────────────────────────────────────────────┘
```

---

## ROS2 Layer

### What is ROS2?

**ROS2 (Robot Operating System 2)** is a middleware framework that provides:

- Communication infrastructure between software modules
- Hardware abstraction for different robot platforms
- Reusable libraries for robotics tasks
- Development and debugging tools

### Core Concepts

#### Nodes

A **node** is an independent process performing a specific task.

**Example: WebSocket Node**

```python
class WebSocketServer(Node):
    def __init__(self):
        super().__init__('websocket_server')

        # Create publisher
        self.head_publisher = self.create_publisher(
            Float64,                                        # Message type
            '/model/cori/joint/head_joint/cmd_pos',        # Topic name
            10                                              # Queue size
        )
```

**CORI Nodes:**

- `websocket_server` - Bridges web commands to ROS2
- `manual_controller` - Command-line control interface
- `color_display` - Processes vision data
- `gazebo` - Physics simulation engine

#### Topics

**Topics** are named channels using publish-subscribe pattern:

```
Publisher → [Topic Name] → Subscriber(s)
```

**Key CORI Topics:**

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/model/cori/joint/head_joint/cmd_pos` | `std_msgs/Float64` | Commands | Head position control |
| `/cori/color_detected` | `std_msgs/String` | Status | Color detection output |

#### Messages

**Messages** are strongly-typed data structures:

```python
from std_msgs.msg import Float64

msg = Float64()
msg.data = 0.5  # Radians for joint angle
```

#### Quality of Service (QoS)

**QoS** defines message delivery guarantees:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,    # Guaranteed delivery
    durability=QoSDurabilityPolicy.VOLATILE,      # No message history
    depth=10                                       # Queue size
)
```

### ROS2 Communication Flow

```
┌─────────────────────────────────────────────────┐
│  1. WebSocket Node Receives Command            │
│     {"type": "angle", "data": {"angle": 0.5}}  │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│  2. Create ROS2 Message                         │
│     msg = Float64()                             │
│     msg.data = 0.5                              │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│  3. Publish to Topic                            │
│     self.head_publisher.publish(msg)            │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│  4. DDS Middleware                              │
│     - Serializes message                        │
│     - Discovers subscribers                     │
│     - Transmits via shared memory/UDP           │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│  5. Gazebo Plugin Receives Message              │
│     joint->SetPosition(0.5)                     │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│  6. Physics Engine Updates Robot                │
│     Head rotates to 28.6° left                  │
└─────────────────────────────────────────────────┘
```

### ROS2 Commands Reference

```bash
# List all active nodes
ros2 node list

# View node details
ros2 node info /websocket_server

# List all topics
ros2 topic list

# Monitor topic messages
ros2 topic echo /model/cori/joint/head_joint/cmd_pos

# Get topic info (subscribers, publishers, QoS)
ros2 topic info /model/cori/joint/head_joint/cmd_pos

# Publish test message
ros2 topic pub /model/cori/joint/head_joint/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"

# Run manual control
ros2 run cori_control manual_control look left
```

---

## Network Layer

### Port Architecture

| Port | Service | Scope | Protocol | Purpose |
|------|---------|-------|----------|---------|
| **80** | Nginx | Public | HTTP | Main web access |
| **8080** | Nginx | Public | HTTP | Alternative (ISP-proof) |
| **8091** | Web UI | Internal | HTTP | Serves HTML/CSS/JS |
| **8767** | WebSocket | Internal | WebSocket | Real-time commands |
| **8000** | FastAPI | Internal | HTTP | REST API |

### Nginx Reverse Proxy

**Why Use a Reverse Proxy?**

A reverse proxy provides:

- **Single Entry Point** - One URL for multiple backend services
- **Security** - Centralized authentication and access control
- **Load Balancing** - Distribute traffic across servers
- **SSL/TLS Termination** - Handle encryption in one place
- **URL Routing** - Map paths to different services

#### Configuration Breakdown

```nginx
server {
    # Listen on multiple ports and protocols
    listen 80;              # IPv4 HTTP
    listen [::]:80;         # IPv6 HTTP
    listen 8080;            # Alternative HTTP
    listen [::]:8080;       # IPv6 alternative

    server_name localhost _;

    # Security Headers
    add_header X-Frame-Options DENY;                          # Prevent clickjacking
    add_header X-Content-Type-Options nosniff;                # Prevent MIME sniffing
    add_header X-XSS-Protection "1; mode=block";              # Enable XSS protection
    add_header Referrer-Policy strict-origin-when-cross-origin;

    # Global Authentication - Single Login
    auth_basic "CORI Robot Access";
    auth_basic_user_file /etc/nginx/.htpasswd;

    # Route 1: Web Interface
    location / {
        proxy_pass http://127.0.0.1:8091/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    # Route 2: API Endpoints
    location /api/ {
        proxy_pass http://127.0.0.1:8000/api/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        proxy_connect_timeout 5s;
        proxy_send_timeout 10s;
        proxy_read_timeout 10s;
    }

    # Route 3: WebSocket Connection
    location /ws {
        proxy_pass http://127.0.0.1:8767;

        # WebSocket upgrade headers (REQUIRED)
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # Longer timeouts for persistent connections
        proxy_connect_timeout 7s;
        proxy_send_timeout 300s;
        proxy_read_timeout 300s;
    }

    # Block hidden files
    location ~ /\. {
        deny all;
        access_log off;
        log_not_found off;
    }

    # Block sensitive file types
    location ~* \.(sql|log|conf)$ {
        deny all;
    }
}
```

### WebSocket Protocol

**HTTP vs WebSocket Comparison:**

| Feature | HTTP | WebSocket |
|---------|------|-----------|
| **Connection** | Request-response | Persistent |
| **Direction** | Client initiates | Bidirectional |
| **Overhead** | High (headers per request) | Low (single handshake) |
| **Latency** | Higher | Lower |
| **Use Case** | Static content, APIs | Real-time, streaming |

**WebSocket Handshake:**

```http
# 1. Client initiates upgrade
GET /ws HTTP/1.1
Host: 74.134.87.123
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13

# 2. Server accepts
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=

# 3. Connection established - messages flow both ways
```

**CORI WebSocket Messages:**

```javascript
// Client → Server: Command
{
    "type": "angle",
    "data": {
        "angle": 0.5
    }
}

// Server → Client: Feedback
{
    "type": "feedback",
    "data": {
        "processing_time_ms": 12,
        "status": "success"
    }
}
```

### HTTP Basic Authentication

**Authentication Flow:**

```
1. Client requests: http://74.134.87.123/

2. Server responds:
   HTTP/1.1 401 Unauthorized
   WWW-Authenticate: Basic realm="CORI Robot Access"

3. Browser shows login prompt

4. User enters credentials: CORI / connect

5. Browser encodes to Base64:
   "CORI:connect" → "Q09SSTpjb25uZWN0"

6. All subsequent requests include:
   Authorization: Basic Q09SSTpjb25uZWN0

7. Nginx validates against /etc/nginx/.htpasswd

8. Valid → proxy to backend
   Invalid → 401 Unauthorized
```

**Create/Update Password:**

```bash
# Create password file
sudo htpasswd -c /etc/nginx/.htpasswd CORI

# Update existing password
sudo htpasswd /etc/nginx/.htpasswd CORI
```

---

## Web Interface Layer

### Technology Stack

| Technology | Purpose | Version |
|------------|---------|---------|
| **HTML5** | Structure and semantics | Latest |
| **CSS3** | Styling, animations, responsive design | Latest |
| **JavaScript** | Logic and interactivity | ES6+ |
| **WebSocket API** | Real-time communication | Native |

**No frameworks required** - Lightweight, fast, and simple.

### Responsive Design

**Mobile-First Approach:**

```css
/* Base styles for mobile */
.container {
    max-width: 1200px;
    padding: 20px;
}

/* Tablet and larger */
@media (max-width: 768px) {
    .sliders-grid {
        grid-template-columns: 1fr;  /* Single column */
    }
}

/* Small phones */
@media (max-width: 480px) {
    .header h1 {
        font-size: 2rem;  /* Smaller headings */
    }
}
```

### Touch-Optimized Controls

```css
/* Large touch targets (minimum 44px for iOS, 48px for Android) */
.slider {
    height: 48px;
}

.slider::-webkit-slider-thumb {
    width: 40px;
    height: 40px;
}

/* Prevent touch selection issues */
* {
    -webkit-tap-highlight-color: transparent;
    user-select: none;
}
```

### JavaScript Controller

```javascript
class CORIController {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.setupEventListeners();
        this.loadSettings();
    }

    connect() {
        // Auto-detect WebSocket URL
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const wsUrl = `${protocol}//${host}/ws`;

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            this.connected = true;
            this.updateUI();
        };

        this.ws.onmessage = (event) => {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        };

        this.ws.onclose = () => {
            this.connected = false;
            this.updateUI();
        };
    }

    sendAngle(angle) {
        if (!this.connected) return;

        this.ws.send(JSON.stringify({
            type: 'angle',
            data: { angle: parseFloat(angle) }
        }));
    }
}
```

### Slider Event Handling

**Two-Event Approach for Optimal Performance:**

```javascript
// 1. Visual feedback during drag (no network traffic)
slider.addEventListener('input', (e) => {
    const angle = parseFloat(e.target.value);
    displayValue.textContent = `${(angle * 57.2958).toFixed(1)}°`;
});

// 2. Send command only when released (reduces network load)
slider.addEventListener('change', (e) => {
    const angle = -parseFloat(e.target.value);
    controller.sendAngle(angle);
});
```

**Benefits:**

- Smooth visual feedback while dragging
- Minimal network traffic (only send final value)
- Better battery life on mobile devices
- Lower server load

### Auto-Detection of WebSocket URL

```javascript
if (window.location.hostname === 'localhost') {
    // Local development - direct connection
    wsUrl = 'ws://localhost:8767';
} else {
    // Remote access - use nginx proxy
    wsUrl = `ws://${window.location.host}/ws`;
}
```

**Examples:**

| Access Method | HTTP URL | WebSocket URL |
|---------------|----------|---------------|
| Localhost | `http://localhost` | `ws://localhost:8767` |
| Local Network | `http://192.168.1.26` | `ws://192.168.1.26/ws` |
| Internet | `http://74.134.87.123` | `ws://74.134.87.123/ws` |

---

## Complete Data Flow

### End-to-End Command Example

**Scenario:** User moves CORI's head left using phone

```
┌───────────────────────────────────────────────────────────┐
│ STEP 1: User Interaction                                  │
│ • User drags slider on phone                              │
│ • JavaScript updates display: "28.6°"                     │
│ • User releases slider                                     │
│ • Event fires: sendAngle(0.5)                             │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 2: WebSocket Message                                 │
│ • Create JSON: {"type": "angle", "data": {"angle": 0.5}} │
│ • WebSocket.send() transmits                              │
│ • Protocol: WebSocket (ws://)                             │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 3: Network Transit                                   │
│ • Cellular tower → Internet backbone                      │
│ • Routes to public IP: 74.134.87.123                      │
│ • Latency: ~50-150ms                                      │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 4: Router Port Forwarding                            │
│ • Router NAT table lookup                                 │
│ • External :80 → Internal 192.168.1.26:80                │
│ • Packet forwarded to CORI machine                        │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 5: Nginx Reverse Proxy                               │
│ • Receives on port 80                                     │
│ • Validates authentication (already logged in)            │
│ • Matches route: /ws                                      │
│ • Proxies to: http://127.0.0.1:8767                      │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 6: WebSocket Server (ROS2 Node)                      │
│ • Receives message on port 8767                           │
│ • Parses JSON                                              │
│ • Validates: type="angle", angle=0.5                      │
│ • Creates ROS2 message: Float64(data=0.5)                │
│ • Publishes to topic                                       │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 7: ROS2 Middleware (DDS)                             │
│ • Serializes to CDR format                                │
│ • Discovers subscribers                                    │
│ • Transmits via shared memory                             │
│ • QoS: Reliable delivery                                   │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 8: Gazebo Joint Controller                           │
│ • Subscriber callback triggered                           │
│ • Extracts angle: 0.5 radians                             │
│ • Applies to joint: joint->SetPosition(0.5)               │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 9: Physics & Rendering                               │
│ • Physics engine updates robot pose                       │
│ • Render loop displays movement                           │
│ • CORI's head rotates 28.6° left                         │
└───────────────────────────────────────────────────────────┘
                            ↓
┌───────────────────────────────────────────────────────────┐
│ STEP 10: Feedback Response                                │
│ • WebSocket sends: {"type": "feedback", ...}             │
│ • Travels back through nginx → router → internet         │
│ • Phone displays: "12 ms"                                 │
└───────────────────────────────────────────────────────────┘
```

### Latency Breakdown

**Local Network (WiFi):**

| Stage | Time | Notes |
|-------|------|-------|
| UI → WebSocket | 1-2ms | Browser processing |
| WebSocket → ROS2 | 1-5ms | IPC overhead |
| ROS2 → Gazebo | 5-10ms | DDS + physics |
| Feedback return | 1-2ms | WebSocket response |
| **Total** | **10-20ms** | Imperceptible delay |

**Internet (Cellular):**

| Stage | Time | Notes |
|-------|------|-------|
| UI → WebSocket | 50-150ms | Cellular latency |
| WebSocket → ROS2 | 1-5ms | IPC overhead |
| ROS2 → Gazebo | 5-10ms | DDS + physics |
| Feedback return | 50-150ms | Return trip |
| **Total** | **100-300ms** | Noticeable but usable |

---

## Remote Access Configuration

### Network Address Translation (NAT)

**Problem:** Private IP addresses aren't routable on the internet

**Solution:** NAT allows multiple devices to share one public IP

```
Internet (Public IPs)
        ↕
[Router: 74.134.87.123] ← Single public IP
        ↕
Home Network (Private IPs - Not internet routable)
├─ 192.168.1.1   (Router internal interface)
├─ 192.168.1.26  (CORI machine) ← Target
├─ 192.168.1.50  (Laptop)
└─ 192.168.1.100 (Phone on WiFi)
```

### Port Forwarding Setup

**Router Configuration:**

| Setting | Value | Notes |
|---------|-------|-------|
| **Service Name** | CORI | Description only |
| **External Port** | 80 | Port on public IP |
| **Internal IP** | 192.168.1.26 | CORI machine |
| **Internal Port** | 80 | Port on CORI machine |
| **Protocol** | TCP | HTTP/WebSocket protocol |
| **Enabled** | ✓ Yes | Must be active |

**Flow:**

```
External Request: 74.134.87.123:80
         ↓
Router NAT Table Lookup
         ↓
Internal Forward: 192.168.1.26:80
         ↓
CORI Machine Receives Request
```

### NAT Loopback Issue

**Problem:** Testing from inside your own network

Many routers don't support **NAT hairpinning** (loopback):

```
CORI Machine (192.168.1.26)
        ↓ tries to access
Public IP (74.134.87.123)
        ↓
Router: "That's me, but I can't loop back!"
        ↓
Connection refused ❌
```

**Solution:** Use appropriate URL based on location

| Location | Use This URL |
|----------|--------------|
| Same WiFi network | `http://192.168.1.26` |
| Different network | `http://74.134.87.123` |
| Cellular data | `http://74.134.87.123` |

### Dynamic DNS (DDNS)

**Problem:** Public IP addresses can change

**When IPs Change:**

- Router reboots
- ISP DHCP lease expires
- ISP maintenance

**Solution:** Use Dynamic DNS

```
1. Sign up: duckdns.org, no-ip.com, dynu.com
2. Create hostname: mycori.duckdns.org
3. Install DDNS client on CORI
4. Client updates DNS automatically:
   mycori.duckdns.org → 74.134.87.123
5. IP changes? Client updates again:
   mycori.duckdns.org → 74.134.87.200
```

**Setup Example (DuckDNS):**

```bash
#!/bin/bash
# /home/juptegraph/update_ddns.sh

DOMAIN="mycori"
TOKEN="your-token-here"

curl "https://www.duckdns.org/update?domains=$DOMAIN&token=$TOKEN"
```

**Automate with cron:**

```bash
# Update every 5 minutes
*/5 * * * * /home/juptegraph/update_ddns.sh
```

### Firewall Configuration

**Ubuntu UFW:**

```bash
# Check status
sudo ufw status

# Allow HTTP ports
sudo ufw allow 80/tcp
sudo ufw allow 8080/tcp

# Allow from specific IP (more secure)
sudo ufw allow from 74.134.87.123 to any port 80

# Enable firewall
sudo ufw enable
```

---

## Security Architecture

### Defense in Depth Strategy

**Multiple Layers of Protection:**

```
Layer 1: Router Firewall
         ↓ Blocks most ports
Layer 2: Port Forwarding Rules
         ↓ Only 80/8080 accessible
Layer 3: Nginx Reverse Proxy
         ↓ Hides internal services
Layer 4: HTTP Basic Authentication
         ↓ Username/password required
Layer 5: Input Validation
         ↓ Rejects malicious data
Layer 6: ROS2 Permissions
         ↓ Sandboxed execution
```

### Security Headers

```nginx
# Prevents clickjacking attacks
add_header X-Frame-Options DENY;

# Prevents MIME type sniffing
add_header X-Content-Type-Options nosniff;

# Enables browser XSS protection
add_header X-XSS-Protection "1; mode=block";

# Controls referrer information leakage
add_header Referrer-Policy strict-origin-when-cross-origin;
```

### HTTP Basic Auth Limitations

**Security Concerns:**

| Issue | Severity | Mitigation |
|-------|----------|------------|
| Credentials in every request | Medium | Use HTTPS |
| Base64 is not encryption | High | Use HTTPS |
| No session timeout | Low | Implement token auth |
| Vulnerable to MITM | Critical | Use HTTPS |

**Recommendation:** Always use HTTPS for internet access

### Upgrade to HTTPS/TLS

**Install Let's Encrypt Certificate:**

```bash
# Install Certbot
sudo apt install certbot python3-certbot-nginx

# Obtain certificate (requires domain name)
sudo certbot --nginx -d mycori.duckdns.org

# Auto-renewal
sudo systemctl enable certbot.timer
sudo systemctl start certbot.timer
```

**Nginx HTTPS Configuration:**

```nginx
server {
    listen 443 ssl http2;
    listen [::]:443 ssl http2;

    server_name mycori.duckdns.org;

    # SSL Certificates
    ssl_certificate /etc/letsencrypt/live/mycori.duckdns.org/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/mycori.duckdns.org/privkey.pem;

    # Strong SSL Settings
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers HIGH:!aNULL:!MD5;
    ssl_prefer_server_ciphers on;

    # HSTS (HTTP Strict Transport Security)
    add_header Strict-Transport-Security "max-age=31536000" always;

    # ... rest of configuration
}

# Redirect HTTP to HTTPS
server {
    listen 80;
    listen [::]:80;
    server_name mycori.duckdns.org;
    return 301 https://$server_name$request_uri;
}
```

### Additional Security Measures

#### Rate Limiting

```nginx
# Prevent brute force attacks
limit_req_zone $binary_remote_addr zone=cori_limit:10m rate=10r/s;

location / {
    limit_req zone=cori_limit burst=20 nodelay;
}
```

#### IP Whitelisting

```nginx
# Allow only trusted IPs
geo $allowed_ip {
    default 0;
    74.134.87.123 1;     # Your static IP
    10.0.0.0/8 1;        # Corporate network
}

server {
    if ($allowed_ip = 0) {
        return 403;
    }
}
```

#### VPN Access (Most Secure)

```
Internet
    ↓
[VPN Server] ← Only this exposed
    ↓ encrypted tunnel
CORI (not directly accessible)
```

**Benefits:**

- All traffic encrypted
- CORI not directly exposed to internet
- Access control at VPN level
- Can use multiple services securely

---

## Troubleshooting Guide

### Connection Issues

#### Problem: "Connection Refused" from Internet

**Symptoms:** Works locally, fails remotely

**Diagnostic Commands:**

```bash
# Check if ports are listening
ss -tlnp | grep -E ":(80|8080|8767)"

# Verify nginx is running
sudo systemctl status nginx

# Test from CORI machine (may fail due to NAT loopback)
curl -I http://74.134.87.123

# Check public IP
curl ifconfig.me
```

**Common Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| Port forwarding not set up | Configure router: External 80 → 192.168.1.26:80 |
| ISP blocking port 80 | Use port 8080 instead |
| Firewall blocking | `sudo ufw allow 80/tcp` |
| Nginx not on all interfaces | Config: `listen 0.0.0.0:80;` |
| Dynamic IP changed | Check: `curl ifconfig.me` |

#### Problem: WebSocket Won't Connect

**Symptoms:** UI loads but "Connect" button fails

**Diagnostic Commands:**

```bash
# Check WebSocket server
ss -tlnp | grep 8767

# Test WebSocket port
curl -I http://localhost:8767

# Check nginx WebSocket proxy
sudo tail -f /var/log/nginx/error.log
```

**Common Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| WebSocket server not running | Start ROS2 WebSocket node |
| Missing upgrade headers | Add to nginx: `proxy_set_header Upgrade $http_upgrade;` |
| Wrong WebSocket URL | Use: `ws://74.134.87.123/ws` not `:8767` |
| HTTPS with WS (not WSS) | Use `wss://` for HTTPS pages |

#### Problem: Commands Don't Reach Robot

**Symptoms:** WebSocket connected, controls don't work

**Diagnostic Commands:**

```bash
# Monitor ROS2 topic
ros2 topic echo /model/cori/joint/head_joint/cmd_pos

# Check Gazebo is running
ps aux | grep gazebo

# List ROS2 nodes
ros2 node list

# Verify publisher
ros2 topic info /model/cori/joint/head_joint/cmd_pos
```

**Common Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| ROS2 node not publishing | Check node logs for errors |
| Wrong topic name | Verify with `ros2 topic list` |
| Gazebo not subscribed | Check topic info for subscribers |
| QoS mismatch | Ensure compatible QoS policies |

### Performance Issues

#### Problem: High Latency (>500ms)

**Diagnostic:**

```javascript
// Browser console test
const start = Date.now();
ws.send(JSON.stringify({type: "ping"}));
ws.onmessage = () => {
    console.log("Latency:", Date.now() - start, "ms");
};
```

**Common Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| Poor internet connection | Test: `ping 8.8.8.8` (should be <100ms) |
| Cellular network congestion | Try different location or WiFi |
| Router overloaded | Reboot router |
| CPU overload on CORI | Check: `htop` |

### Debugging Commands

```bash
# ═══ Network Diagnostics ═══

# Get public IP
curl ifconfig.me

# Test external access
curl -I http://74.134.87.123:80

# Check listening ports
ss -tlnp

# Test nginx config
sudo nginx -t

# Reload nginx
sudo systemctl reload nginx

# View logs
sudo tail -f /var/log/nginx/access.log
sudo tail -f /var/log/nginx/error.log


# ═══ ROS2 Diagnostics ═══

# List nodes
ros2 node list

# List topics
ros2 topic list

# Monitor topic
ros2 topic echo /model/cori/joint/head_joint/cmd_pos

# Topic frequency
ros2 topic hz /model/cori/joint/head_joint/cmd_pos

# Topic info
ros2 topic info /model/cori/joint/head_joint/cmd_pos

# Test publish
ros2 topic pub /model/cori/joint/head_joint/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"


# ═══ System Diagnostics ═══

# CPU/Memory usage
htop

# Disk space
df -h

# System logs
journalctl -f

# Service status
sudo systemctl status nginx
```

---

## Quick Reference

### Connection Information

| Access Type | HTTP URL | WebSocket URL |
|-------------|----------|---------------|
| **Localhost** | `http://localhost` | `ws://localhost:8767` |
| **Local Network** | `http://192.168.1.26` | `ws://192.168.1.26/ws` |
| **Internet** | `http://74.134.87.123` | `ws://74.134.87.123/ws` |

**Authentication:**

- **Username:** `CORI`
- **Password:** `connect`
- **Method:** HTTP Basic Auth (browser popup)

### Port Reference

| Port | Service | Scope | Purpose |
|------|---------|-------|---------|
| **80** | Nginx | Public | Main HTTP access |
| **8080** | Nginx | Public | Alternative (ISP-proof) |
| **8091** | Web UI | Internal | Serves interface files |
| **8767** | WebSocket | Internal | Real-time commands |
| **8000** | FastAPI | Internal | REST API |

### ROS2 Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/model/cori/joint/head_joint/cmd_pos` | `std_msgs/Float64` | Head position command |
| `/cori/color_detected` | `std_msgs/String` | Color detection output |

### File Locations

```
CORI Project Structure:
/home/juptegraph/Workspaces/Robotics/Projects/CORI/

├── cori_ws/                                  # ROS2 workspace
│   ├── nginx_cori.conf                       # Nginx config source
│   ├── src/
│   │   ├── cori_hardware/
│   │   │   └── cori_hardware/
│   │   │       └── index.html                # Web interface
│   │   ├── cori_control/
│   │   │   └── cori_control/
│   │   │       └── manual_control.py         # CLI control
│   │   └── ...
│   └── install/                              # Built packages

├── docs/                                     # Documentation
│   └── CORI_System_Architecture.md          # This file

/etc/nginx/
├── sites-available/cori                      # Nginx config (deployed)
├── sites-enabled/cori                        # Symlink to above
└── .htpasswd                                 # Authentication file
```

### Essential Commands

```bash
# ═══ Build & Deploy ═══

# Build CORI packages
source /opt/ros/humble/setup.bash
colcon build

# Deploy nginx config
sudo cp cori_ws/nginx_cori.conf /etc/nginx/sites-available/cori
sudo ln -sf /etc/nginx/sites-available/cori /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl reload nginx

# Update authentication
sudo htpasswd /etc/nginx/.htpasswd CORI


# ═══ Running Services ═══

# Launch Gazebo simulation
ros2 launch cori_simulation gazebo.launch.py

# Start WebSocket server
ros2 run cori_hardware websocket_server

# Start Web UI server (if separate)
python3 -m http.server 8091

# Manual CLI control
ros2 run cori_control manual_control look left


# ═══ Networking ═══

# Check public IP
curl ifconfig.me

# Test remote access
curl -I http://74.134.87.123

# Allow firewall ports
sudo ufw allow 80/tcp
sudo ufw allow 8080/tcp

# View nginx logs
sudo tail -f /var/log/nginx/access.log
```

---

## Conclusion

The CORI system demonstrates a complete **real-time robotic control architecture** that seamlessly integrates:

- **ROS2** - Professional robotics middleware with reliable communication
- **Web Technologies** - Modern, accessible interface for any device
- **Network Infrastructure** - Secure remote access from anywhere
- **Mobile-First Design** - Optimized for touch and responsive layouts

### Key Achievements

✅ **Low Latency** - 10-20ms local, 100-300ms remote
✅ **Simple Interface** - Single login, auto-detected URLs
✅ **Secure** - Multi-layer defense with optional HTTPS
✅ **Scalable** - Production-ready architecture
✅ **Universal Access** - Works on any device with a browser

### Use Cases

This architecture is suitable for:

- Educational robotics platforms
- Remote robot monitoring and control
- Telepresence systems
- Industrial automation dashboards
- IoT device management
- Research demonstrations

---

**Project Information:**

- **Author:** Johnathan Uptegraph
- **System:** CORI (Character-Oriented Robot Interface)
- **ROS2 Version:** Humble Hawksbill
- **Documentation Date:** November 6, 2025

---

*For questions, issues, or contributions, refer to the project repository.*
