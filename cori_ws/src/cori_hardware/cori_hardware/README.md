# CORI Remote Control - GitHub Pages Deployment

Host your CORI robot control interface on GitHub Pages for global access.

## 🚀 Quick Setup

### Step 1: Create GitHub Repository
1. Create a new repository on GitHub (e.g., `cori-remote-control`)
2. Enable GitHub Pages in repository settings
3. Set source to "Deploy from a branch" → `main` branch

### Step 2: Upload Files
Copy `index.html` to your GitHub repository root:
```bash
git clone https://github.com/your-username/cori-remote-control.git
cd cori-remote-control
cp /path/to/cori_ws/src/cori_hardware/cori_hardware/index.html .
git add index.html
git commit -m "Add CORI remote control interface"
git push origin main
```

### Step 3: Access Your Control Interface
Your control interface will be available at:
```
https://your-username.github.io/cori-remote-control/
```

## 🌐 Remote Access Configuration

### Robot Setup (Your Local Machine)
1. **Start the real-time control server:**
   ```bash
   cd /path/to/cori_ws
   source install/setup.bash
   ros2 run cori_hardware realtime_web_control
   ```

2. **Configure your router/firewall:**
   - Port forward `8765` (WebSocket) to your robot's IP
   - Or use a tunnel service like ngrok:
   ```bash
   ngrok tcp 8765
   ```

### Web Interface Configuration
1. Open your GitHub Pages URL
2. Enter your robot's WebSocket URL:
   - **Local network**: `ws://your-robot-ip:8765`
   - **Port forwarded**: `ws://your-public-ip:8765`
   - **ngrok tunnel**: `ws://your-ngrok-url:port`
3. Click "Connect" or use "Auto-Detect"

## 🔧 Advanced Configuration

### Custom Domain
1. Add a `CNAME` file to your repository:
   ```
   robot.yourdomain.com
   ```
2. Configure DNS A/CNAME records to point to GitHub Pages

### HTTPS/WSS for Secure Connections
For HTTPS GitHub Pages, use WSS (secure WebSocket):
```javascript
// In your robot setup, use a reverse proxy like nginx:
wss://your-domain.com/ws
```

### Multiple Robots
Modify the auto-detect function to include your robot IPs:
```javascript
const commonUrls = [
    'ws://robot1.local:8765',
    'ws://robot2.local:8765',
    'ws://your-robot-ip:8765'
];
```

## 📱 Mobile Optimization

The interface is fully responsive and works on:
- ✅ Desktop browsers
- ✅ Mobile browsers (iOS/Android)
- ✅ Touch devices
- ✅ Gamepad controllers

## 🎮 Control Methods

### Keyboard Controls
- `←/→` - Move head left/right
- `↑` - Center head
- `Space` - Emergency stop
- `C` - Connect/disconnect

### Gamepad Controls
- Left stick X-axis - Head movement
- Any button - Emergency stop

### Touch Controls
- Tap color buttons for preset movements
- Drag angle slider for precise control
- Hardware control buttons

## 🔒 Security Considerations

### Network Security
- Use WSS (secure WebSocket) for public access
- Configure firewall rules to limit access
- Consider VPN for secure remote access

### Authentication (Optional)
Add basic auth to your WebSocket server:
```python
async def handle_client(self, websocket, path):
    # Add authentication logic here
    auth_token = websocket.request_headers.get('Authorization')
    if not self.validate_token(auth_token):
        await websocket.close(code=1008, reason="Unauthorized")
        return
```

## 🐛 Troubleshooting

### Connection Issues
1. **"Connection failed"**: Check robot IP and port
2. **"Connection timeout"**: Verify firewall/router settings
3. **"WebSocket error"**: Ensure robot is running and accessible

### Performance Issues
1. **High latency**: Check network connection quality
2. **Dropped commands**: Verify WebSocket stability
3. **Slow response**: Ensure robot isn't overloaded

### Browser Issues
1. **HTTPS/WSS required**: Some browsers block unsecure WebSockets on HTTPS pages
2. **CORS errors**: Use `origins=None` in WebSocket server
3. **Mobile issues**: Test on different devices/browsers

## 📊 Monitoring

### Real-time Metrics
The interface displays:
- Connection status
- Command latency (ms)
- Command count
- Robot status
- Error messages

### Logging
Check robot logs for detailed information:
```bash
ros2 run cori_hardware realtime_web_control --ros-args --log-level debug
```

## 🔄 Updates

To update your GitHub Pages interface:
1. Modify `index.html` locally
2. Test changes locally
3. Push to GitHub repository
4. Changes appear automatically on GitHub Pages

## 🤝 Example Repository Structure
```
cori-remote-control/
├── index.html          # Main control interface
├── README.md           # Documentation
├── CNAME              # Custom domain (optional)
└── assets/            # Additional assets (optional)
    ├── favicon.ico
    └── manifest.json
```

## 🌟 Features

- **Zero-latency control** via WebSocket
- **Global access** from any device
- **Auto-detection** of robot
- **Real-time feedback** and status
- **Multi-platform support** (desktop/mobile)
- **Gamepad integration**
- **Emergency stop** safety
- **Responsive design**

Your CORI robot is now controllable from anywhere in the world! 🌍🤖