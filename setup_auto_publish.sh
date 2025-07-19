#!/bin/bash
# CORI Robot Auto-Publish Setup Script

echo "🤖 Setting up CORI Robot auto-publishing to juptegraph.dev"

# Install required Python packages
echo "📦 Installing Python dependencies..."
pip3 install requests

# Setup environment variables
echo "🔑 Setting up GitHub token..."
echo "Please enter your GitHub Personal Access Token:"
read -s github_token

# Create environment file
cat > ~/.cori_env << EOF
export GITHUB_TOKEN=$github_token
export ROBOT_NAME=CORI
export WEBSITE_REPO=juptegraph.dev
export GITHUB_USERNAME=your-username
EOF

echo "source ~/.cori_env" >> ~/.bashrc

# Setup systemd service
echo "⚙️ Setting up systemd service..."
sudo cp systemd/cori-status-publisher.service /etc/systemd/system/
sudo sed -i "s/your_github_token_here/$github_token/g" /etc/systemd/system/cori-status-publisher.service
sudo sed -i "s|/home/juptegraph|$HOME|g" /etc/systemd/system/cori-status-publisher.service

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable cori-status-publisher.service
sudo systemctl start cori-status-publisher.service

echo "✅ Setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Create a GitHub Personal Access Token with 'repo' permissions"
echo "2. Update the GitHub username in robot_status_publisher.py"
echo "3. Add the robot widget to your juptegraph.dev website"
echo "4. Test the service: sudo systemctl status cori-status-publisher"
echo ""
echo "🌐 Widget URL: file://$(pwd)/web/robot-widget.html"