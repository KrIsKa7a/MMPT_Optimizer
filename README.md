# MMPT_Optimizer
This repository contains the code and the project for the [***IMM 2024 Hackathon***](https://www.industrymeetsmakers.com/infineon-pv-optimizer). The goal of the project is to create an optimized algorithm for MPPT of PVs. The project is based on the provided project by ***Infineon*** in their [repository](https://github.com/Infineon/IMM2024). The goal of the project is to extend the provided functionality into fully working hardware device that can achieve MPPT optimization. This will be achieved by applying the following strategy:
- Upgrade of the Incremental Conductance MPPT algorithm which will be executed by the Slow Control Loop
- Smart Mode and Reference Selection Algorithm which will be executed by the Medium Control Loop
- DC-DC Converter Control Algorithm using the provided digital PID regulator which will be executed by the Fast Control Loop
# Workflow Process (Only for developers/contributors)
This repository and the project itself are still in the proof of concept stage. Hence, there can be a lot of changes and issues. <br/>
When you want to clone the repository ***ALWAYS*** clone the code inside the ***develop*** branch. <br/>
Create a local branch to work on your changes. You can commit as much as you want in your local branch. <br/>
When you want to deliver your changes, you need to publish your local branch and create a pull request. In the pull request, it is suggested that you check the checkboxes about deleting the source branch and squashing the branch commits.<br/>
Our goal is to maintain the repository in such way that it always has the following branches:
- ***main*** branch - It will contain the source code of the developed device
- ***develop*** branch - It will always contain the top version of the development source code
- ***rel_vXX*** branches - These branches will contain the stable releases during the development phase. The **XX** stands for the version number
- ***local*** branches - You are free to create as many local branches as you wish. You just need to merge them in the ***develop*** branch.