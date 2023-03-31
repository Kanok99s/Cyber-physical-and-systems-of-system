# 2023-group-09


## Getting started: 

1. To clone this repository, make sure you have an active SSH ke. To check if your SSH key is active, you can use this command: ls ~/.ssh/id_rsa.pub or cat ~/.ssh/id_ed25519.pub 

If you have such file on your pc, then this confirms that you have an active SSH key. 

If you dont have an SSH key then follow these steps:  

Generate an SSH key pair on your local machine by running the following command in your terminal: 

ssh-keygen -t rsa -b 4096 
 

Follow the prompts to generate the key pair. Make sure to leave the passphrase blank for now. 

Copy the contents of the public key file to your clipboard by running the following command: 

cat ~/.ssh/id_rsa.pub | pbcopy 

Log in to your Chalmers GitLab account and go to Settings > SSH Keys. 

Click the "Add SSH key" button. 

Paste the contents of your public key into the "Key" field. 

 

2. To clone the project to the right folder where you want it to be:  

first create a folder: mkdir project 

Navigate to the folder: cd project 

then clone the repository to your local machine using the git clone command: 

git clone git@git.chalmers.se:/courses/dit638/students/2023-group-09 

 

3. Navigate to the root folder of the repository using the cd command: cd 2023-group-09 

 

4. Install and set-up Docker to be able to run the project. You can follow the instructions for installation at (https://docs.docker.com/get-docker/) and on [Docker Compose](https://docs.docker.com/compose/install/). 

 

5. Once you've downloaded docker and docker compose, navigate to the project repository folder in your terminal and run the following command: 

docker-compose up 

6. This will start the OpenDLV-Vehicle-View and the h264 decoder. You will see a message like the following:  

server listening on port: 8081, joining live OD4Session 111, using OD4Session 253 for playback. 

7. Open another terminal, and in same folder, run the following command:  

Make build 

Then to run the project: make run 

## Add your files



```
cd existing_repo
git remote add origin https://git.chalmers.se/courses/dit638/students/2023-group-09.git
git branch -M main
git push -uf origin main
```



***


## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.


## License
For open source projects, say how it is licensed.


## Add Features
* Creating an issue that describes the feature specifically and assign it to one of the developers in the team.
* Start implementing the solution for the feature by writting code.
* Creating a branch with a descriptive name which specifies the functionality of the feature.
* Creating a merge request for reviewing the code with other developers.
* Merging the code after the approval.

## Bug Fixing 
* Creating an issue with a clear description of the error and assign one of the developers.
* Identifying the steps which lead to the error in the code and fix the issue.
* Create a merge request for reviewing the solution.
* After the code has been reviewed, we can merge.

## Commit Messages
* Keep the message concise and to the point.
* Provide more context in the comment section if necessary.
* Use the imperative mood.
* Follow a consistent format for your commit messages throughout the project.

## Authors and acknowledgment
* Akuen Akoi Deng 
* Kanokwan Haesatith
* Nazli Moghaddam
* Cynthia Tarwireyi
* Marwa Selwaye
