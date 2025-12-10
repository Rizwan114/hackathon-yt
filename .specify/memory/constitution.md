<!-- Sync Impact Report:
Version change: 1.0.0 → 2.0.0
List of modified principles: Updated all principles to reflect new project requirements
Added sections: Technical Standards, Content Standards, RAG Chatbot requirements, ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action systems, Capstone project
Removed sections: Previous beginner-focused content
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# Unified Book + Embedded RAG Chatbot for Physical AI & Humanoid Robotics Constitution

## Vision
To create a comprehensive, technically accurate educational resource that integrates a complete book on Physical AI & Humanoid Robotics with an embedded RAG-powered chatbot, enabling students and practitioners to learn, implement, and interact with cutting-edge robotics technologies through a unified, reproducible framework.

## Core Principles

### I. Spec-Driven Accuracy
All content must adhere strictly to project specs, including Docusaurus structure, Spec-Kit Plus conventions, Claude Code workflows, and integrations with OpenAI Agents/ChatKit SDKs. No content should deviate from established specifications without explicit approval.

### II. Technical Precision
Explanations must be technically correct for an audience familiar with AI, robotics, simulation, and software engineering. All code examples, configurations, and procedures must reflect current best practices and real-world implementations in 2024-2025.

### III. Educational Clarity
Writing should be accessible to upper-division computer science and robotics students (Flesch-Kincaid Grade 12–14 level). Complex concepts must be explained with appropriate technical depth while maintaining clarity and understanding.

### IV. Reproducibility
All instructions for code, deployment, ROS 2 setups, Gazebo/Unity simulation workflows, Isaac configurations, and chatbot/RAG pipeline must be complete and executable as written. Every example must work in a clean environment without missing dependencies or steps.

### V. Modularity & Extensibility
Book chapters, code modules, and chatbot components must follow a consistent structure enabling future expansion. Each module should function independently while integrating seamlessly with the broader system.

### VI. Integration Focus
All components must work together as a unified system. The RAG chatbot must accurately ground responses in book content, ROS 2 components must integrate with simulation environments, and Isaac AI systems must connect with physical robot implementations.

## Content Standards

Coverage must include all required modules:

- ROS 2 robotic nervous system
- Gazebo & Unity digital twin simulation
- NVIDIA Isaac AI-robot brain
- Vision-Language-Action systems
- Capstone: Autonomous Humanoid

Each chapter must include:
- Concept explanation
- Practical code examples
- Integration with other modules
- Exercises/labs
- Evaluation checkpoints

All robotics content must reflect current practice in 2024–2025.

## Technical Standards

### Book Framework
Must be written and structured in Docusaurus with proper navigation, search, and responsive design.

### Tooling
Must use Spec-Kit Plus + Claude Code for generation, editing, and maintenance of all content and code.

### Deployment
Must deploy to GitHub Pages with a reproducible pipeline that can be executed from a clean environment.

### RAG Chatbot Requirements
Built using OpenAI Agents/ChatKit SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier.

Must support:
- Answering questions about the book
- Grounding responses strictly in retrieved sections
- Restricting answers to user-selected text when applicable

### Code Quality Standards
- All examples must run, compile, or launch as stated
- ROS 2 examples must use rclpy
- Gazebo and Unity scenes must include minimal reproducible configurations
- Isaac workflows must be testable in a standard GPU environment

### Documentation Standards
- Every code snippet must be validated
- All factual claims must cite authoritative technical sources (ROS docs, NVIDIA Isaac docs, robotics papers)
- Citation format: APA
- Minimum 20 sources, at least 40% peer-reviewed

## Constraints

- Book Length: Equivalent of 25,000–40,000 words across Docusaurus pages
- All chapters must be spec-complete and free of placeholder text
- Zero hallucinated APIs, functions, ROS packages, or Isaac features
- RAG chatbot responses must not invent content not found in book text
- All code examples must run in standard Python environments (3.8+) with common robotics libraries
- Dependencies must be well-maintained and available on standard package managers

## Brand Voice

- Technically authoritative yet approachable
- Precise and accurate in technical descriptions
- Encouraging of experimentation and learning
- Professional tone appropriate for advanced students
- Clear distinction between theoretical concepts and practical implementations
- Inclusive language that welcomes diverse technical backgrounds

## Development Workflow

- All content undergoes technical accuracy verification before publication
- Code examples must be tested in clean environments
- Documentation structure follows Docusaurus best practices
- Regular integration testing of all components
- Validation of RAG chatbot retrieval accuracy
- Peer review by domain experts for technical correctness

## Success Criteria

### Book Completeness
- All modules covered in depth with practical examples
- Capstone project fully implemented and documented from end to end
- Build + deploy pipeline functions from a clean environment
- All code examples execute as documented

### Chatbot Functionality
- Correct grounding: answers based only on book text or selected text
- Retrieval accuracy verified through evaluation tests
- Database (Neon) and vector store (Qdrant) integration fully functional
- Natural conversation flow with appropriate context management

### Integration Quality
- Seamless interaction between book content and chatbot responses
- Cross-module functionality working as specified
- Deployment pipeline executes without manual intervention

## Governance

This constitution serves as the guiding document for all Physical AI & Humanoid Robotics book and chatbot development. All content, code, and documentation must align with these principles. Changes to this constitution require explicit approval and documentation of the reasoning. All contributors must acknowledge and follow these principles when creating content for the project.

**Version**: 2.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10