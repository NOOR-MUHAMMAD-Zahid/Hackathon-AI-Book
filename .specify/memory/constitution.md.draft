<!--
SYNC IMPACT REPORT
Version change: N/A → 1.0.0 (Initial version)
Added sections:
- Spec-Driven Development principle
- Technical Accuracy and Implementation Correctness principle
- Clarity for Developers and Advanced Learners principle
- Reproducibility principle
- Modularity principle
- RAG Chatbot Excellence principle
- Technical Standards and Constraints section
- Development Workflow and Quality Assurance section
Modified principles: N/A (all new)
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ reviewed, Constitution Check section aligns with principles
- .specify/templates/spec-template.md: ✅ reviewed, aligns with new requirements
- .specify/templates/tasks-template.md: ✅ reviewed, task organization supports new principles
- .specify/templates/phr-template.prompt.md: ✅ reviewed, no conflicts
- .specify/templates/adr-template.md: ✅ reviewed, no conflicts
Follow-up TODOs: None
-->
# AI/Spec-driven Technical Book Constitution

## Core Principles

### Spec-Driven Development
All work must be guided by explicit specifications. Implementation follows detailed specs in the form of feature requirements, architecture decisions, and testable tasks. Every change must have a clear specification that defines scope, requirements, and acceptance criteria.

### Technical Accuracy and Implementation Correctness
All code examples, configurations, and procedures must be technically accurate and verifiable. Code examples must be runnable and version-pinned to ensure reproducibility. No theoretical or untested solutions allowed in the final deliverables.

### Clarity for Developers and Advanced Learners
Content must be written with clear, precise language suitable for developers and advanced learners. Explanations should be comprehensive yet accessible, with practical examples that demonstrate real-world applications.

### Reproducibility
All steps, code, and configurations must be fully traceable and reproducible. From repository clone to deployment, the entire process must be replicable by others. All dependencies, versions, and environment requirements must be explicitly documented.

### Modularity
The book, chatbot, and infrastructure must be cleanly separated with well-defined interfaces. Each component should be independently developable, testable, and deployable while maintaining tight integration where required.

### RAG Chatbot Excellence
The Retrieval-Augmented Generation chatbot must answer questions from full book content accurately, prevent hallucinations outside retrieved context, and respond using only user-selected text when specified. The retrieval logic, embeddings, and chunking strategy must be fully documented.

## Technical Standards and Constraints

Documentation framework: Docusaurus for the book content. Content format: All chapters and pages written in Markdown (.md). Source control: GitHub with deployment via GitHub Pages. Book authored using Claude Code + Spec-Kit Plus with all architectural decisions explicitly documented. Code examples must be runnable and version-pinned.

RAG chatbot architecture: Retrieval-Augmented Generation (RAG) using OpenAI Agents / ChatKit SDKs, FastAPI backend, Qdrant Cloud (Free Tier) vector store, and Neon Serverless Postgres database. Deployment targets GitHub Pages (book) with cloud-hosted API (chatbot).

## Development Workflow and Quality Assurance

All work follows the Spec-Driven Development methodology with explicit specifications (spec.md), architectural plans (plan.md), and testable tasks (tasks.md). Implementation must strictly follow the spec, with deviations requiring specification updates first. Code reviews must verify compliance with all constitutional principles. Automated testing and validation are required for all features.

## Governance

This constitution supersedes all other development practices for this project. All team members must follow these principles. Amendments require explicit documentation of the change, approval from project leadership, and a migration plan for existing work. All pull requests and code reviews must verify constitutional compliance. Changes to this constitution require a formal amendment process with justification and impact assessment.

**Version**: 1.0.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-22