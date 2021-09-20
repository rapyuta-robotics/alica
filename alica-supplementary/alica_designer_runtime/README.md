# The ALICA Designer runtime

- To run the designer use the docker-compose.yml file:
    $ docker-compose up
- The designer is web based & runs on the browser. After executing the above command, navigate to http://localhost:3000/ to run the designer
- Create/Modify the plans & export them to the your application's etc folder
- To generate the source code you need to download https://drive.google.com/file/d/1xAQ84NJSTOohR93QHBrysxz31v5sXZh2/view?usp=sharing & extract the .zip archive into the alica-supplementary/alica_designer_runtime folder
- Use the generate.sh script to generate the source code
    $ ./generate.sh <path_to_folder_that_contains_etc>
- generate.sh generates the code in <path_to_folder_that_contains_etc>/Expr