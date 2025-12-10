# Tasks: Physical AI Book — Module 1: The Robotic Nervous System (ROS 2)

**Feature**: Physical AI Book — Module 1: The Robotic Nervous System (ROS 2)
**Branch**: `001-ros2-robotic-nervous`
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)
**Generated**: 2025-12-10

## Implementation Strategy

MVP approach: Start with User Story 1 (ROS 2 Nodes) as the core functionality, then incrementally add other user stories. Each user story should be independently testable and deliver value on its own.

## Phase 1: Setup

Setup tasks to initialize the project structure and dependencies.

- [x] T001 Create Docusaurus project structure in docs/ directory
- [x] T002 Initialize backend directory with FastAPI project structure
- [x] T003 Set up GitHub Actions workflow for automated deployment
- [x] T004 Configure project dependencies in package.json and requirements.txt
- [x] T005 Create examples directory structure for code examples

## Phase 2: Foundational

Foundational tasks that block all user stories.

- [x] T006 Create Docusaurus configuration with proper navigation for module-1-ros2
- [x] T007 Set up backend dependencies (FastAPI, OpenAI SDK, Qdrant client, Neon driver)
- [x] T008 Create basic RAG system structure in backend/rag/
- [x] T009 Create database connection utilities for chat session management
- [x] T0010 Create initial URDF validation utilities
- [x] T011 Set up API routing structure for chat endpoints

## Phase 3: [US1] Create and Understand ROS 2 Nodes

User Story 1 - Create functional ROS 2 nodes to understand the fundamental communication model. Priority: P1

**Goal**: Students can create functional ROS 2 nodes, publish/subscribe to topics, and implement services

**Independent Test**: Student creates a publisher node that sends messages and a subscriber node that receives them, demonstrating basic communication pattern

- [x] T012 [US1] Create chapter-1-intro.md explaining the robotic nervous system concept
- [x] T013 [US1] Create chapter-2-fundamentals.md covering ROS 2 node lifecycle and conventions
- [x] T014 [P] [US1] Create basic publisher node example in examples/python/ros2_nodes/simple_publisher.py
- [x] T015 [P] [US1] Create basic subscriber node example in examples/python/ros2_nodes/simple_subscriber.py
- [x] T016 [US1] Add topic communication content to chapter-2-fundamentals.md
- [x] T017 [P] [US1] Create service example in examples/python/ros2_nodes/simple_service.py
- [x] T018 [P] [US1] Create service client example in examples/python/ros2_nodes/simple_service_client.py
- [x] T019 [US1] Add QoS models content to chapter-2-fundamentals.md
- [x] T020 [US1] Add debugging tools content to chapter-2-fundamentals.md
- [x] T021 [US1] Validate ROS 2 examples in clean environment

## Phase 4: [US3] Create and Validate Humanoid Robot URDF Model

User Story 3 - Write and validate a complete URDF model of a humanoid robot. Priority: P1

**Goal**: Students can write and validate a complete URDF model with links, joints, sensors, and actuators

**Independent Test**: Student creates a URDF file for a simple humanoid model and validates it using check_urdf tool, then visualizes it in RViz2

- [x] T022 [US3] Create chapter-4-urdf.md explaining URDF structure and concepts
- [x] T023 [P] [US3] Create basic humanoid URDF model in examples/urdf/humanoid_model.urdf
- [x] T024 [US3] Add links and joints content to chapter-4-urdf.md
- [x] T025 [P] [US3] Add sensors to URDF model (camera, IMU, LiDAR) in examples/urdf/humanoid_model.urdf
- [x] T026 [US3] Add URDF validation content to chapter-4-urdf.md
- [x] T027 [US3] Test URDF model with check_urdf validation
- [x] T028 [US3] Add export instructions for Gazebo and Isaac to chapter-4-urdf.md

## Phase 5: [US2] Build Python Agent to ROS Bridge

User Story 2 - Integrate Python-based AI agents with ROS 2 controllers. Priority: P2

**Goal**: Students can bridge Python-based AI agents to ROS 2 controllers using rclpy

**Independent Test**: Student creates a Python script that sends commands to a ROS 2 service or action server and receives sensor data back

- [ ] T029 [US2] Create chapter-3-rclpy.md covering Python-based robot controllers
- [ ] T030 [P] [US2] Create Python agent example that sends commands to ROS 2 in examples/python/rclpy_agents/agent_controller.py
- [ ] T031 [US2] Add AI agent integration content to chapter-3-rclpy.md
- [ ] T032 [P] [US2] Create action server example for humanoid tasks in examples/python/rclpy_agents/action_server.py
- [ ] T033 [P] [US2] Create action client example in examples/python/rclpy_agents/action_client.py
- [ ] T034 [US2] Add action server content to chapter-3-rclpy.md
- [ ] T035 [US2] Add safety mechanisms (rate limiting, timeouts) to chapter-3-rclpy.md
- [ ] T036 [P] [US2] Add safety constraints to Python agent examples

## Phase 6: [US4] Implement End-to-End Control Pipeline

User Story 4 - Implement a minimal humanoid control pipeline. Priority: P1

**Goal**: Students can implement a control pipeline from perception → planning → actuation

**Independent Test**: Student implements a simple control loop that takes sensor input (perception), processes it (planning), and sends commands to actuators (actuation)

- [ ] T037 [US4] Create chapter-5-project.md for the mini-project
- [ ] T038 [P] [US4] Create multi-node ROS 2 system example in examples/python/ros2_nodes/control_pipeline.py
- [ ] T039 [US4] Add perception → planning → actuation content to chapter-5-project.md
- [ ] T040 [P] [US4] Integrate URDF model with control pipeline in examples/python/ros2_nodes/control_pipeline.py
- [ ] T041 [P] [US4] Create Python agent → ROS bridge example in examples/python/rclpy_agents/integrated_bridge.py
- [ ] T042 [US4] Document the complete mini-project requirements in chapter-5-project.md
- [ ] T043 [US4] Test complete control pipeline integration

## Phase 7: RAG System Integration

RAG system tasks to enable chatbot functionality for the book content.

- [ ] T044 Create embedding module for book content in backend/rag/embedding.py
- [ ] T045 Create retrieval module for content search in backend/rag/retrieval.py
- [ ] T046 Implement chat endpoint in backend/main.py following API contract
- [ ] T047 Create chat processing module in backend/rag/chat.py
- [ ] T048 Implement content validation endpoint in backend/main.py
- [ ] T049 Create content retrieval endpoint in backend/main.py
- [ ] T050 Add RAG integration components to Docusaurus site

## Phase 8: Polish & Cross-Cutting Concerns

Final tasks to complete the module and ensure quality.

- [ ] T051 Review and validate all code examples for ROS 2 Humble compatibility
- [ ] T052 Ensure all content is between 4,000-7,000 words total
- [ ] T053 Add APA citations to all technical sources in chapters
- [ ] T054 Validate all URDF models and test with check_urdf
- [ ] T055 Test Docusaurus build process and fix any issues
- [ ] T056 Run RAG system end-to-end tests with book content
- [ ] T057 Verify all examples are reproducible and free of hallucinated APIs
- [ ] T058 Add accessibility features to documentation (proper headings, alt text)
- [ ] T059 Create quality validation scripts for future content updates
- [ ] T060 Final review and proofreading of all chapter content

## Dependencies

User Story Completion Order:
1. User Story 1 (P1) - ROS 2 Nodes (foundational)
2. User Story 3 (P1) - URDF Model (foundational)
3. User Story 2 (P2) - Python Agent Integration (builds on US1)
4. User Story 4 (P1) - End-to-End Pipeline (integrates all previous stories)

## Parallel Execution Examples

Per User Story:
- **US1**: T014 (publisher) and T015 (subscriber) can run in parallel
- **US3**: T023 (URDF model) and T025 (sensors) can run in parallel
- **US2**: T030 (agent controller) and T032 (action server) can run in parallel
- **US4**: T038 (control pipeline) and T040 (URDF integration) can run in parallel