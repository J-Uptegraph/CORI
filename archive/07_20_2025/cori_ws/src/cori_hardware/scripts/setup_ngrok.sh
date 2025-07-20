#!/bin/bash
# Quick setup script for ngrok tunnel to ESP32

echo "🚀 Setting up ngrok tunnel for ESP32 remote access..."

# Check if ngrok is installed
if ! command -v ngrok &> /dev/null; then
    echo "❌ ngrok not found. Installing..."
    
    # Download and install ngrok
    wget -q https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-amd64.tgz
    tar -xzf ngrok-v3-stable-linux-amd64.tgz
    sudo mv ngrok /usr/local/bin/
    rm ngrok-v3-stable-linux-amd64.tgz
    
    echo "✅ ngrok installed"
fi

echo "📝 Setup Instructions:"
echo "1. Sign up at https://ngrok.com and get your authtoken"
echo "2. Run: ngrok config add-authtoken YOUR_TOKEN_HERE"
echo "3. Connect your ESP32 to WiFi access point (CORI_HEAD_CONTROLLER)"
echo "4. Run: ngrok http 192.168.4.1:80"
echo "5. Share the public URL with your friend!"

echo ""
echo "🔄 To start tunnel now (after setup):"
echo "ngrok http 192.168.4.1:80"