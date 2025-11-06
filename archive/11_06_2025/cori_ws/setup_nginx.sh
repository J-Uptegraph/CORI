#!/bin/bash
set -e

echo "🔧 Setting up CORI Nginx Reverse Proxy..."

# Get credentials from environment variables or prompt user
USERNAME="${CORI_USERNAME:-}"
PASSWORD="${CORI_PASSWORD:-}"

if [ -z "$USERNAME" ] || [ -z "$PASSWORD" ]; then
    echo "Enter username for CORI access:"
    read -r USERNAME
    echo "Enter password for $USERNAME:"
    read -rs PASSWORD
fi

echo "Setting up CORI access with username: $USERNAME"

# Create htpasswd file
echo "$PASSWORD" | sudo htpasswd -ic /etc/nginx/.htpasswd "$USERNAME"

# Copy nginx config
sudo cp nginx_cori.conf /etc/nginx/sites-available/cori
sudo ln -sf /etc/nginx/sites-available/cori /etc/nginx/sites-enabled/cori

# Remove default nginx site
sudo rm -f /etc/nginx/sites-enabled/default

# Test nginx config
sudo nginx -t

# Restart nginx
sudo systemctl restart nginx
sudo systemctl enable nginx

echo "✅ Nginx configured successfully!"
echo "🌐 Access your robot at: http://your-server-ip"
echo "🔐 Username: $USERNAME"
echo "📱 WebSocket endpoint: ws://your-server-ip/ws"

# Show current status
sudo systemctl status nginx --no-pager -l