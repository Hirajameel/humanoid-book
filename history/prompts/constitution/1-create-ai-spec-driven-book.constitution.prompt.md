---
id: 1
title: Create AI Spec Driven Book
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution,spec-driven,rag,book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

/sp.constitution

Project: Physical AI & Humanoid Robotics

Purpose:
Create a technical book using Docusaurus, written with Claude Code and Spec-Kit Plus, deployed to GitHub Pages, with an embedded RAG chatbot that answers questions grounded strictly in the book's content.

Core Principles:
- Specification-first development
- Technical accuracy and clarity
- Reproducibility and maintainability
- Modular architecture
- No hallucinations (retrieval-grounded responses only)

Deliverables:
1. Book:
   - Written in Markdown for Docusaurus
   - Version-controlled on GitHub
   - Deployed to GitHub Pages
   - Clear structure, code examples, and setup guides

2. RAG Chatbot:
   - Embedded in the book UI
   - FastAPI backend
   - OpenAI Agents / ChatKit SDKs
   - Qdrant Cloud (Free Tier) for vectors
   - Neon Serverless Postgres for metadata/state
   - Supports full-book and user-selected-text Q&A

Standards:
- All claims traceable to book content
- Runnable or clearly labeled pseudocode
- Clear separation of documentation, backend, retrieval, and data layers
- Free-tier infrastructure only

Constraints:
- Follow Spec-Kit Plus conventions
- No hardcoded secrets
- Book and chatbot must stay in sync

Success Criteria:
- Book deploys successfully
- Chatbot answers only from retrieved content
- Selected-text-only Q&A works correctly
- End-to-end reproducible setup

## Response snapshot

Created the project constitution file (.specify/memory/constitution.md) with the following principles:
1. Specification-First Development
2. Technical Accuracy and Clarity
3. Reproducibility and Maintainability
4. Modular Architecture
5. No Hallucinations (Retrieval-Grounded Responses Only)
6. Free-Tier Infrastructure Focus

The constitution includes sections for additional constraints, development workflow, and governance rules as specified in the project requirements.

## Outcome

- ✅ Impact: Created initial project constitution with all required principles
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Proceed with project planning and specification
- 🧠 Reflection: Constitution properly captures the core principles of the Physical AI & Humanoid Robotics project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin project specification