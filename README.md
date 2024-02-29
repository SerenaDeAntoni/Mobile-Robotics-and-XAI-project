


<h1>Mobile-Robotics-and-XAI-project</h1>
<div>
<b>A solution of the RockSample problem</b>:
<br/>
a python implementation of a <b>ROS</b> node that uses the <b>Clingo API</b>
to send the initial conditions in ASP format, which calculates a plan 
of action considering as good only the rocks at a distance less
than a given threshold. The robot reaches the goal matching 
each action with a motion primitive and moving with the actions
of the plan in sequence, being careful to not collide with the rocks.
</div>

<img src="agent.gif" />

<h2>Requirements</h2>
<div>
    <ul>
        <li><b>rclpy</b>, a ROS Client Library for the Python language, <a href="https://github.com/ros2/rclpy">https://github.com/ros2/rclpy</a></li>
        <li><b>clingo API</b>, an ASP framework <a href="https://potassco.org/clingo/python-api/5.4/">https://potassco.org/clingo/python-api/5.4/</a></li>
        <li><b>Unity</b> and <b>ROS2</b> are required, we suggest to follow this repository to install and configure the environment <a href="https://gitlab.com/TrottiFrancesco/            mobile_robotics_lab">https://gitlab.com/TrottiFrancesco/mobile_robotics_lab</a></li>
    </ul>
</div>

<h2>Files usage</h2>
<div>
    <ul>
        <li><b>project_exam_grid.unity</b> and <b>project_exam_grid.unity.meta</b>, are the the arena files configuration, containing the agent and the rocks (obstacles)</li>
        <li><b>plan_and_exec.py</b> is the core program to run</li>
    </ul>
</div>

<h2>Program Steps</h2>
<div>
    The program is defined by 3 main steps:
    <ol>
    <li>Lidar phase</li>
    <li>Clingo phase</li>
    <li>Motion phase</li>
    </ol>

<h3>Lidar Phase</h3>
    <p> At the start the agent percives via lidars rocks in enviroment,
    providing distances and angles at which they are detected.
    </p>

<h3>Clingo Phase</h3>
    <p> When rocks distances are computed, they are passed to <b>Clingo</b>,
        which returns the action plan to execute in order to reach each
        rock.
    </p>

<h3>Motion Phase</h3>
    <p> The allowed actions are <b>NORTH</b>, <b>SOUTH</b>, <b>EAST</b>,
        <b>WEST</b> and <b>SAMPLE</b>.
        The first 4 actions are divided in 2 subactions:
        <ol>
            <li><b>Rotation</b>, until we are in the same target orientation</li>
            <li><b>Move foreward</b>, until we don't reach the target, avoiding collision with it</li>
        </ol>

</div>








