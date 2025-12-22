---
id: 1
title: add-constitution-for-ai-book
stage: constitution
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Noor Muhammad
command: sp.constitution
labels: [constitution, spec-driven-development, ai-book, rag-chatbot]
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

```text
- /sp.constitution

Project: AI/Spec-driven technical book with integrated RAG chatbot

Core principles:
- Spec-driven development (all work guided by explicit specifications)
- Technical accuracy and implementation correctness
- Clarity for developers and advanced learners
- Reproducibility (all steps, code, and configs fully traceable)
- Modularity (book, chatbot, and infrastructure cleanly separated)

Key standards:
- Documentation framework: Docusaurus
- Content format: All chapters and pages written in Markdown (.md)
- Source control: GitHub (deploy via GitHub Pages)
- Book authored using Claude Code + Spec-Kit Plus
- All architectural decisions explicitly documented
- Code examples must be runnable and version-pinned

RAG chatbot standards:
- Architecture: Retrieval-Augmented Generation (RAG)
- SDKs: OpenAI Agents / ChatKit
- Backend: FastAPI
- Vector store: Qdrant Cloud (Free Tier)
- Database: Neon Serverless Postgres
- Chatbot must:
  - Answer questions from full book content
  - Answer questions using ONLY user-selected text when specified
  - Prevent hallucinations outside retrieved context
- Retrieval logic, embeddings, and chunking strategy fully documented

Constraints:
- Deployment target: GitHub Pages (book) + cloud-hosted API (chatbot)
- Infrastructure must use free or free-tier services where possible
- All configuration files, schemas, and scripts included in repository
- No proprietary content without clear attribution

Success criteria:
- Book successfully deployed and publicly accessible
- RAG chatbot embedded and functional within the book UI
- Accurate, context-grounded responses verified against book content
- Full reproducibility from repository clone to deployment
- Specification compliance validated via Spec-Kit Plus
```

## Response snapshot

The project constitution has been successfully created and updated at `.specify/memory/constitution.md`. The new constitution establishes 6 core principles: Spec-Driven Development, Technical Accuracy and Implementation Correctness, Clarity for Developers and Advanced Learners, Reproducibility, Modularity, and RAG Chatbot Excellence. It also includes sections on Technical Standards and Constraints, Development Workflow and Quality Assurance, and Governance. The constitution version is 1.0.0, ratified on 2025-12-22. All dependent templates were reviewed for consistency and no issues were found.

## Outcome

- ✅ Impact: Initial project constitution created, establishing governance and principles for AI/Spec-driven technical book project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive constitution
- 🔁 Next prompts: Plan and spec creation for book content and RAG chatbot
- 🧠 Reflection: Constitution properly captures all specified requirements and constraints for the technical book and RAG chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - all requirements met
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin feature specification for book content management