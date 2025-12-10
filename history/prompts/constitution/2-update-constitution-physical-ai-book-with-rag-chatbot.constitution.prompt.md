---
id: 2
title: Update Constitution for Physical AI Book with RAG Chatbot
stage: constitution
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, documentation, physical-ai, rag-chatbot, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Unified Book + Embedded RAG Chatbot for Physical AI & Humanoid Robotics

Core Principles

Spec-Driven Accuracy:
All content must adhere strictly to the project specs, including Docusaurus structure, Spec-Kit Plus conventions, Claude Code workflows, and integrations with OpenAI Agents/ChatKit SDKs.

Technical Precision:
Explanations must be technically correct for an audience familiar with AI, robotics, simulation, and software engineering.

Educational Clarity:
Writing should be accessible to upper-division computer science and robotics students (Flesch-Kincaid Grade 12–14 level).

Reproducibility:
All instructions for code, deployment, ROS 2 setups, Gazebo/Unity simulation workflows, Isaac configurations, and chatbot/RAG pipeline must be complete and executable as written.

Modularity & Extensibility:
Book chapters, code modules, and chatbot components must follow a consistent structure enabling future expansion.

Key Standards
Content Standards

Coverage must include all modules:

ROS 2 robotic nervous system

Gazebo & Unity digital twin simulation

NVIDIA Isaac AI-robot brain

Vision-Language-Action systems

Capstone: Autonomous Humanoid

Each chapter must include:

Concept explanation

Practical code examples

Integration with other modules

Exercises/labs

Evaluation checkpoints

All robotics content must reflect current practice in 2024–2025.

Technical Standards

Book Framework: Must be written and structured in Docusaurus.

Tooling: Must use Spec-Kit Plus + Claude Code for generation, editing, and maintenance.

Deployment: Must deploy to GitHub Pages with a reproducible pipeline.

RAG Chatbot:

Built using OpenAI Agents/ChatKit SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier.

Must support:

answering questions about the book,

grounding responses strictly in retrieved sections,

restricting answers to user-selected text when applicable.

Code Quality:

All examples must run, compile, or launch as stated.

ROS 2 examples must use rclpy.

Gazebo and Unity scenes must include minimal reproducible configurations.

Isaac workflows must be testable in a standard GPU environment.

Documentation Standards

Every code snippet must be validated.

All factual claims must cite authoritative technical sources (ROS docs, NVIDIA Isaac docs, robotics papers).

Citation format: APA.

Minimum 20 sources, at least 40% peer-reviewed.

Constraints

Book Length: Equivalent of 25,000–40,000 words across Docusaurus pages.

All chapters must be spec-complete and free of placeholder text.

Zero hallucinated APIs, functions, ROS packages, or Isaac features.

RAG chatbot responses must not invent content not found in book text.

Success Criteria

Book Completeness

All modules covered in depth.

Capstone project fully implemented and documented from end to end.

Build + deploy pipeline functions from a clean environment.

Chatbot Functionality

Correct grounding: answers based only on book text or selected text.

Retrieval accuracy verified through evaluation tests.

Database (Neon) a

## Response snapshot

Updated the constitution document for the Physical AI book project to reflect the comprehensive requirements for a unified book with embedded RAG chatbot for Physical AI & Humanoid Robotics. The updated constitution includes detailed core principles (Spec-Driven Accuracy, Technical Precision, Educational Clarity, Reproducibility, Modularity & Extensibility, Integration Focus), content standards covering all required modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action, Capstone), technical standards for the book framework, tooling, deployment, RAG chatbot, and code quality, constraints, brand voice, development workflow, and success criteria.

## Outcome

- ✅ Impact: Constitution updated to version 2.0.0 with comprehensive requirements for Physical AI book + RAG chatbot project
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Begin implementing book content and RAG chatbot following updated constitution
- 🧠 Reflection: Constitution now properly reflects the complex, multi-component nature of the Physical AI & Humanoid Robotics project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): begin implementing first module based on constitution requirements