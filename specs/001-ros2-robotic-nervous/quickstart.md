# Quickstart Guide: Physical AI Book — Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites

- ROS 2 Humble Hawksbill or newer installed
- Python 3.8 or higher
- Node.js 16 or higher for Docusaurus
- Git for version control
- Access to OpenAI API (for RAG chatbot)
- GitHub account for deployment

## Setup Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up ROS 2 Environment
```bash
# Source ROS 2 setup (adjust for your installation)
source /opt/ros/humble/setup.bash

# Create a workspace for examples
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
pip install rclpy
pip install ament-copyright ament-flake8 ament-pep257
```

### 4. Set up Docusaurus Documentation
```bash
cd docs
npm install
```

### 5. Set up Backend for RAG System
```bash
cd backend
pip install -r requirements.txt
```

## Running the Documentation Locally

### 1. Start Docusaurus Development Server
```bash
cd docs
npm start
```
This will start the documentation site at http://localhost:3000

### 2. Verify Chapter Content
Navigate to `/module-1-ros2/` to view the ROS 2 module chapters.

## Running Code Examples

### 1. Navigate to Examples Directory
```bash
cd examples/python/ros2_nodes
```

### 2. Run a Basic Publisher/Subscriber Example
```bash
# Terminal 1 - Start the publisher
python3 simple_publisher.py

# Terminal 2 - Start the subscriber
python3 simple_subscriber.py
```

### 3. Validate URDF Models
```bash
# Install URDF validation tools if not already installed
sudo apt install ros-humble-urdfdom-tools

# Validate a URDF file
check_urdf examples/urdf/humanoid_model.urdf

# Visualize in RViz2
ros2 run rviz2 rviz2
```

## Running the RAG Chatbot Backend

### 1. Set Environment Variables
```bash
export OPENAI_API_KEY="your-api-key"
export QDRANT_URL="your-qdrant-url"
export QDRANT_API_KEY="your-qdrant-api-key"
export NEON_DATABASE_URL="your-neon-db-url"
```

### 2. Start the Backend Server
```bash
cd backend
uvicorn main:app --reload --port 8000
```

### 3. Test the API
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What are ROS 2 nodes?",
    "module_id": "module-1-ros2"
  }'
```

## Creating New Content

### 1. Add a New Chapter
Create a new Markdown file in `docs/module-1-ros2/`:
```bash
touch docs/module-1-ros2/chapter-X-topic.md
```

### 2. Add Code Examples
Place new Python examples in `examples/python/`:
```bash
mkdir -p examples/python/new_topic
# Add your Python files to this directory
```

### 3. Add URDF Models
Place new URDF files in `examples/urdf/`:
```bash
# Add your URDF files to this directory
# Validate them with check_urdf
```

## Validation Checklist

Before committing changes, ensure:

- [ ] All code examples run successfully in a clean ROS 2 environment
- [ ] URDF models validate without errors using `check_urdf`
- [ ] Docusaurus builds without errors (`npm run build`)
- [ ] All examples use ROS 2 Humble or newer APIs only
- [ ] No hallucinated ROS APIs or functions exist in examples
- [ ] APA citations are properly formatted
- [ ] Content is between 4,000-7,000 words for Module 1
- [ ] RAG system correctly retrieves and cites book content

## Deployment

### 1. Build Documentation
```bash
cd docs
npm run build
```

### 2. Deploy to GitHub Pages
The GitHub Actions workflow will automatically deploy when changes are pushed to the main branch:
```bash
git add .
git commit -m "Update Module 1 content"
git push origin main
```

## Troubleshooting

### Common Issues

1. **ROS 2 nodes not communicating**
   - Ensure both terminals have sourced the ROS 2 environment
   - Check that ROS_DOMAIN_ID is the same in both terminals

2. **Docusaurus build errors**
   - Verify all Markdown syntax is correct
   - Check for broken links in the documentation

3. **RAG system not retrieving content**
   - Verify the vector database is properly populated with book content
   - Check API keys and connection settings

### Getting Help
- Check the ROS 2 documentation at https://docs.ros.org/
- Review Docusaurus documentation at https://docusaurus.io/
- Refer to the project specification in `specs/001-ros2-robotic-nervous/spec.md`