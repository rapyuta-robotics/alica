# The ALICA Designer Runtime


## 3. User Interface Overview

### 3.1 Setting up the configs
Located at the top left, the settings menu allows you to configure:
- **GitHub Application ID** (see [GitHub Workflow Setup](#4-github-workflow)).
- **Backend URL** (where the web server runs).
- **Live Debugging WebSocket URL**.

please complete the settings setup by following the instructions here
![settings](./imagessettings.png)

### 3.2 Import & Export Menu
The import/export menu allows importing and exporting plans via:
- **File System**: Import/export plans from/to a local directory.
- **Zip**: Import/export plans via a zip file.
- **GitHub**: Import/export plans to/from a Git repository.
- **Logout**: Sign out of GitHub.

For details, see [Section 5: Import & Export](#5-import-and-export-plans).

![import_export](./imagesimport_export.png)

### 3.3 Login
Clicking the login button opens the GitHub authentication page. After login, the button changes to display your GitHub username.

![login_page](./imageslogin_page.png)

---

## 4. Creating and Managing ALICA Elements

The right-side menu allows the creation of ALICA elements:
- **Plans**
- **Behaviors**
- **Plan Types**
- **Configurations**
- **Tasks & Task Repositories**
- **Roles & Role Repositories**
- **Conditions**

Each element type has specific properties. Below are descriptions of the key elements:

### 4.1 Plans
Plans define sequences of actions executed by autonomous agents.

![create_plan](./imagescreate_plan.png)

- **Utility Threshold**: Minimum required utility improvement for reassignment.
- **Frequency**: Execution rate of the plan's `run` method.
- **Master Plan**: Marks the plan as the main plan.

### 4.2 Behaviors
Reusable components defining specific agent actions.

![create_behaviour](./imagescreate_behaviour.png)

- **Event-Driven**: Runs manually instead of at a fixed rate.
- **Frequency**: Execution rate per second.
- **Deferring**: Initial delay before the first execution.

### 4.3 Tasks & Task Repositories

Tasks define unit objectives.

![create_task](./imagescreate_task.png)

Task repositories store related tasks together.

### 4.4 Roles & Role Sets

Roles define different agent responsibilities.

![create_role](./imagescreate_role.png)

- **Role Set**: Stores related roles.
- **Default Priority**: Used if no priority is set for a task.
- **Default Role Set**: Marks this set as the default.

![create_roleset](./imagescreate_roleset.png)

---

## 5. Import and Export Plans

ALICA Designer supports importing and exporting plans using:
1. **Local File System** (Native Mode required).
2. **Zip Files**.
3. **GitHub Repositories**.

### 5.1 File System Import/Export

> **Prerequisites:**
> - `NATIVE_MODE=true` in `config.env`.
> - `NATIVE_IMPORT_EXPORT_PATH` set to a valid directory.

#### Export
Click **File System → Export** to save plans locally.

![import_export_fs](./imagesimport_export_fs.png)

#### Import
Click **File System → Import** to load plans from the local directory.

---

### 5.2 Zip Import/Export
Plans can be exported/imported as zip files.

#### Export
Click **Zip → Export to Zip** to download a `.zip` file containing the plans.

![import_export_zip](./imagesimport_export_zip.png)

#### Import
Click **Zip → Import from Zip** to upload plans from a zip file.

---

### 5.3 GitHub Import/Export
Plans can be imported/exported via a GitHub repository.

#### Export
Click **Git → Export to GitHub** to push plans to a repository branch.

#### Import
Click **Git → Import from GitHub** to pull plans from a repository.

---

## 6. Plan Creation & Editing

### 6.1 Selection Menu
Lists all created elements for quick selection.

![selection_menu](./imagesselection_menu.png)

### 6.2 Element Properties
Displays settings for the selected element, including:
- **General properties**
- **Conditions**
- **Variables**
- **Usages**
- **Plans**
- **Variable Bindings**
- **Parameters**
- **Roles**
- **Task Priorities**
- **Blackboard**

### 6.3 Plan Editor
The main workspace for creating plans. It includes:
- Selection Tool
- Entry Point Tool
- State Tools (Normal, Success, Failure)
- Synchronization Tools

![plan_creation_tools](./imagesplan_creation_tools.png)

#### 6.3.1 Adding Entry Points
Define agent starting positions.

![create_entry_point](./imagescreate_entry_point.png)

#### 6.3.2 Creating States
Different states for a plan:
- **Normal State**
- **Failure State**
- **Success State**

#### 6.3.3 Defining Transitions
Create transitions by connecting states.

![transition_condition](./imagestransition_condition.png)

#### 6.3.4 Synchronization
Coordinate agent actions using synchronization nodes.

![synchronization](./imagessynchronization.png)

#### 6.3.5 Adding Elements
Drag & drop behaviors, configurations, plan types, or plans into states.

#### 6.3.6 Deleting Elements
Select an element and press **Delete**.

---

## 7. Conclusion

This guide provides a structured approach to using the ALICA Designer. By following these steps, users can efficiently create, modify, and manage autonomous behavior plans. For further details, refer to the [ALICA documentation](https://rapyuta-robotics.github.io/alica/).

