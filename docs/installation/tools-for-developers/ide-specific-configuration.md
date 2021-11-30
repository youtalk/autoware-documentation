# IDE Specific Configuration

Any IDE or text editor can be used to develop Autoware.Auto. Here, the configuration directives for some can be found.

## Visual Studio Code

Visual Studio Code is a popular IDE. It is a free and open source software.

### Install Visual Studio Code

To install VS Code, follow [official document](https://code.visualstudio.com/docs/setup/linux#_installation).

#### Install VS Code Extension

- [C/C++ for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)

### Setup workspace

After you installed Autoware.Core, you can run the following command to setup the workspace.

```{bash}
cd ~/autoware.proj
python3 scripts/repo2workspace.py autoware.proj.repos
```

### Launching Visual Studio Code

You can launch VS Code by the following command.

```{bash}
cd ~/autoware.proj
source install/setup.bash
code autoware.proj.code-workspace
```

### Building

Terminal within VS Code can be used to build the Autoware.Core.

@ref building "Building Section" has the fundamental information that will be useful in here.

In order to be able to debug the code with VS Code, the code should be compiled with either `Debug` or`RelWithDebInfo` flags. `RelWithDebInfo` flag can be used most of the time without problems.

```{bash}
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to <package_name>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select <package_name>
```

### Running and debugging the nodes

Now anything that can be run with `ros2 run package_name executable_name <param1> <param2>...` can be debugged.

The nodes that are run with `ros2 launch` command cannot be debugged with following method although it should be fairly simple to [have VS Code Attach to a Process](https://code.visualstudio.com/docs/editor/debugging#_launch-versus-attach-configurations).

!!! Note

      If you see an error saying: `ptrace: Operation not permitted`, follow [the instructions](https://code.visualstudio.com/docs/cpp/cpp-debug#_debugging)

#### Example for running the "point_cloud_filter_transform_nodes" from the perception/filters in ROS2 Foxy:\*\*

Normally this node can be run with following commands:

```{bash}
source ~/autoware.proj/install/setup.bash
ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args -r __ns:=/lidar_front --params-file ~/autoware.proj/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_xyzi
```

To be able to run and/or debug this file with VS Code, do the following:

1. Follow [the debugging instructions](https://code.visualstudio.com/docs/editor/debugging) and create a `launch.json` file.

2. Add the following lines to the `launch.json` file.

   ```{json}
      {
      "version": "0.2.0",
      "configurations": [
         {
            "name": "launch node",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/install/point_cloud_filter_transform_nodes/lib/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node_exe",
            "args": ["--ros-args", "-r", "__ns:=/lidar_front", "--params-file", "${workspaceFolder}/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml", "-r", "__node:=filter_transform_vlp16_front", "-r", "points_in:=/lidar_front/points_xyzi"],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
            {
               "text": "-enable-pretty-printing",
               "ignoreFailures": true
            }
            ]
         }
      ]
      }
   ```

3. Save the `launch.json` file.

After saving, it should now be possible to click the Triangle button int the [Run view](https://code.visualstudio.com/docs/editor/debugging#_run-view) to run or debug the application :)

#### Configuring for other nodes

The `program` and `args` can be modified to make it work with any other node.

## CLion {#ide-specific-configuration-clion}

Here, Autoware.Auto was installed @ref installation-no-ade "without ADE" in the "~/projects/AutowareAuto" path.

The key functionality that will make CLion able to index is the use of a [Compilation Database](https://www.jetbrains.com/help/clion/compilation-database.html).

### Launching CLion

You can use the default desktop launcher of the CLion to launch it. Unlike ROS1 CLion configuration, it doesn't need to be run from a terminal where ROS was sourced.

### Building

Terminal within CLion can be used to build the Autoware.Auto.

@ref building "Building Section" has the fundamental information that will be useful in here.

@ref building-compilation-database "Building with compilation database" should be followed to generate the Compilation Database.

In order to be able to debug the code with CLion, the code should be compiled with either `Debug` or`RelWithDebInfo` flags. `RelWithDebInfo` flag can be used most of the time without problems.

```{bash}
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to <package_name>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-select <package_name>
```

Also some `bash` aliases can be set in the `~/.bash_aliases` file to simplify the building process.

```{bash}
alias colcon_build_reldeb="colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
alias colcon_build_reldeb_upto="colcon_build_reldeb --packages-up-to "
alias colcon_build_reldeb_sel="colcon_build_reldeb --packages-select "
```

### Opening up the project in CLion

Once Autoware.Auto is compiled with the commands above, it can be loaded into CLion.

Navigate to `File | Open` in the main menu, then choose `compile_commands.json`(it will be located in `AutowareAuto/build/` folder) file and click Open as Project.

By default, the project root is set to the directory containing the compilation database file which, in our case, is the `AutowareAuto/build/` folder.

#### Change the project root {#ide-specific-configuration-change-project-root}

To change the project root, select `Tools | Compilation Database | Change Project Root` from the main menu, and select the `AutowareAuto` directory from there.

Now CLion's code insight, refactoring, analysis, and navigation are fully available for the project.

To finish things up,

- Project Pane -> right click `install` folder and select `Mark Directory as -> Excluded`
- Project Pane -> right click `src` folder and select `Mark Directory as -> Project Sources and Headers`

And it should look like this:

@image html images/ide-configuration-clion-01-first-run.png "CLion First Run" width=90%

### Running and debugging the nodes

Now anything that can be run with `ros2 run package_name executable_name <param1> <param2>...` can be debugged.

The nodes that are run with `ros2 launch` command cannot be debugged with following method although it should be fairly simple to [have Clion Attach to a Process](https://www.jetbrains.com/help/clion/attaching-to-local-process.html).

#### Example for running the "point_cloud_filter_transform_nodes" from the perception/filters in ROS2 Foxy:\*\*

Normally this node can be run with following commands:

```{bash}
# In my pc, the AutowareAuto is located in ~/projects/
# Please update this path with your configuration

source ~/projects/AutowareAuto/install/setup.bash
ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args -r __ns:=/lidar_front --params-file ~/projects/AutowareAuto/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_xyzi
```

There are some [Path Variables](https://www.jetbrains.com/help/clion/absolute-path-variables.html) in CLion that make it easy to shorten some paths in certain places.

Here `$ContentRoot$` will be used to point to the project root which is `~/projects/AutowareAuto`.

In order for `$ContentRoot$` to work, make sure the @ref ide-specific-configuration-change-project-root "project root is changed" to `AutowareAuto` folder.

And either:

- A file in the Project pane that belongs to the project must be selected.
- A file that belongs to the project must be opened and active.

When actively working on a project, these conditions will mostly be satisfied without an extra effort.

To be able to run and/or debug this file with CLion, do the following:

1. Call Run | Edit Configurations from the menu or Click `Add Configuration...` from top right near build buttons.

1. Click on the plus button on top-left and pick `Custom Build Application`
   @image html images/ide-configuration-clion-02-run-configuration-empty.png "Empty Custom Build Configuration" width=50%

1. In the `Before Launch` list, press the minus button to remove the `Build` from the list.
   @image html images/ide-configuration-clion-03-run-configuration-custom-build-application.png "Custom Build Application" width=50%

1. Click `Configure Custom Build Targets`

1. Click the plus button from top-left to add a custom build target.
   @image html images/ide-configuration-clion-04-custom-build-targets.png "Custom Build Targets" width=50%

   Leave the rest as they are, as this is a dummy target. And click OK.
   @image html images/ide-configuration-clion-05-custom-build-targets-done.png "Custom Build Targets Done" width=50%

1. In the `Custom Build Application` screen select the `Custom Build Target` for `Target:` that you have generated.

1. Executable: `/home/mfc/projects/AutowareAuto/install/point_cloud_filter_transform_nodes/lib/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node_exe` (Absolute path is required here)

1. Program Arguments: `--ros-args -r __ns:=/lidar_front --params-file $ContentRoot$/src/perception/filters/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml -r __node:=filter_transform_vlp16_front -r points_in:=/lidar_front/points_xyzi`

1. (optional) Working Directory: `$ContentRoot$/install/`

1. Environment Variables: `source /home/mfc/projects/AutowareAuto/install/setup.bash` (Absolute path is required here)

In the end it should look like this:

@image html images/ide-configuration-clion-06-run-configuration-done.png "Run/Debug Configurations Done" width=60%

After clicking OK, it should now be possible to click the Triangle or the Bug button to run or debug the application :)

#### Configuring for other nodes

The `Executable` and `Program Arguments` can be modified to make it work with any other node.

### Formatting the code and Documentation related settings

#### Code Style

1. Navigate to `File | Settings` in the main menu.
1. In Settings, navigate to `Editor | Code Style`.
   - Hard wrap at: 100
   - Visual guides at: 100
   - Enable ClangFormat: Checked

It should look like this:

@image html images/ide-configuration-clion-07-code-style.png "Code Style" width=40%

1. In Settings, navigate to `Editor | Code Style | C/C++`.

- On top right click `Set from...`
- Pick Google (Although most will be overwritten by ClangFormat)
- Go to `Code Generation` tab
  - Under Documentation Comments
    - Add \@brief tag: Checked
    - Tag prefix in line comments: \@param
    - Tag prefix in block comments: \@param

It should look like this:

@image html images/ide-configuration-clion-08-code-generation.png "Code Generation" width=40%

#### External Tools

For following commands to work, CLion should be started in a terminal where the current ROS2 is sourced.

1. Navigate to `File | Settings` in the main menu.
1. In Settings, navigate to `Tools | External Tools`.
1. Press the `+` button to add following tools:

   1. ament_cpplint:
      - Name: `ament_cpplint`
      - Program: `ament_cpplint`
      - Arguments: `$FilePath$`
      - Working directory: `$ProjectFileDir$`
        @image html images/ide-configuration-clion-09-external_cpplint.png "ament_cpplint" width=30%
   1. ament_uncrustify:
      - Name: `ament_uncrustify`
      - Program: `ament_uncrustify`
      - Arguments: `--reformat $FilePath$`
      - Working directory: `$ProjectFileDir$`
        @image html images/ide-configuration-clion-10-external_uncrustify.png "ament_uncrustify" width=30%
   1. ament_flake8:
      - Name: `ament_flake8`
      - Program: `ament_flake8`
      - Arguments: `$FilePath$`
      - Working directory: `$ProjectFileDir$`
        @image html images/ide-configuration-clion-11-external_flake8.png "ament_flake8" width=30%

1. Press OK to save settings and exit `Settings` window.
1. Reopen `File | Settings` in the main menu.
1. In Settings, navigate to `Keymap`.
1. Under `External Tools | External Tools` assign hotkeys to these tools.

Once these are set, it is possible to use these hotkeys to automatically run these tools. Also `Reformat Code` shortcut will apply settings of `.clang-format` on the code.

For detailed information about formatting use instructions at @ref contributors-guidelines-formatting "Contributors Guidelines for Formatting"
