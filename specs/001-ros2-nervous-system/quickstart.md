# Quickstart Guide: ROS 2 Course Development

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- Text editor or IDE

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Website Directory
```bash
cd website
```

### 3. Install Dependencies
```bash
npm install
```

### 4. Start Development Server
```bash
npm start
```
This will start a local development server at http://localhost:3000

### 5. Create Course Content
Add your course content to the `docs/` directory following the structure:
```
docs/
├── intro.md
└── ros2-nervous-system/
    ├── index.md
    ├── introduction-to-ros2.md
    ├── ros2-communication.md
    └── urdf-modeling.md
```

### 6. Build for Production
```bash
npm run build
```
This creates a static site in the `build/` directory that can be deployed to GitHub Pages.

### 7. Deploy to GitHub Pages
The site is configured to deploy automatically via GitHub Actions when changes are pushed to the main branch.