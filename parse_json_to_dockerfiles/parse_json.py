"""
JSONファイルを受け取ったら、ホストの数のDockerfileと、それらをまとめて立ち上げるdocker-compose.ymlを作成する
入力例: python parse_json.py examples/topology_example/topology_example.json
"""

import sys
import json
import os
import textwrap

# コマンドラインからJSONファイルのパスを受け取り、そのJSONファイルを取得する
#  (args) -> json 
def load_json_file(args):
    file_path = args[1] # args[0]には実行ファイル名、args[1]にコマンドライン引数が来る
    with open(file_path, "r") as f:
        json_content = json.load(f)

    return json_content


# JSONファイルを受けとり、各ホストに対応するDockerfileを生成する.生成したDockerfileの数だけディレクトリを作り、Dockerfileはその下に置く
#  json -> list[Dockerfile]の生成 
def generate_dockerfiles(json_content):
    output_dir = "../Dockerfiles"
    os.makedirs(output_dir, exist_ok=True)

    with open("../docker_base/Dockerfile", "r") as f:
        docker_base_content = f.read()

    hosts = json_content["hosts"]
    # 各ホストに対し、ノード情報を追記したDockerfileを作成し、Dockerfiles/{ホスト名}/Dockerfile に置く
    for host_dict in hosts:
        dockerfile_content = docker_base_content
        base_command = ""
        host_name = host_dict["host_name"]
        nodes = host_dict["nodes"]

        for index, node in enumerate(nodes):
            node_name = node["node_name"]

            # 最初だけはコマンドの先頭に & をつけない
            if index == 0:
                if node.get("publisher"):
                    publisher_list = node["publisher"]
                    topic_names = ",".join(publisher["topic_name"] for publisher in publisher_list)
                    payload_sizes = ",".join(str(publisher["payload_size"]) for publisher in publisher_list)
                    period_ms = ",".join(str(publisher["period_ms"]) for publisher in publisher_list)

                    additional_command = f". /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node && ./publisher_node_exe --node_name {node_name} --topic_names {topic_names} -s {payload_sizes} -p {period_ms}"
                    base_command += additional_command

                if node.get("subscriber"):
                    subscriber_list = node["subscriber"]
                    topic_names = ",".join(subscriber["topic_name"] for subscriber in subscriber_list)

                    additional_command =f". /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/subscriber_node/lib/subscriber_node && ./subscriber_node --node_name {node_name} --topic_names {topic_names}"
                    base_command += additional_command

                if node.get("intermediate"):
                    publisher_list = node["intermediate"][0]["publisher"]
                    subscriber_list = node["intermediate"][0]["subscriber"]

                    topic_names_pub = ",".join(publisher["topic_name"] for publisher in publisher_list)
                    payload_sizes = ",".join(str(publisher["payload_size"]) for publisher in publisher_list)
                    period_ms = ",".join(str(publisher["period_ms"]) for publisher in publisher_list)
                    topic_names_sub = ",".join(subscriber["topic_name"] for subscriber in subscriber_list)

                    additional_command =f". /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/intermediate_node/lib/intermediate_node && ./intermediate_node --node_name {node_name} --topic_names_pub {topic_names_pub} --topic_names_sub {topic_names_sub} -s {payload_sizes} -p {period_ms}"
                    base_command += additional_command

                continue

            # 複数のノードをバックグラウンドで同時に起動するため、通常は & で結ぶ
            if node.get("publisher"):
                publisher_list = node["publisher"]
                topic_names = ",".join(publisher["topic_name"] for publisher in publisher_list)
                payload_sizes = ",".join(str(publisher["payload_size"]) for publisher in publisher_list)
                period_ms = ",".join(str(publisher["period_ms"]) for publisher in publisher_list)

                additional_command = f" & . /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node && ./publisher_node_exe --node_name {node_name} --topic_names {topic_names} -s {payload_sizes} -p {period_ms}"
                base_command += additional_command

            if node.get("subscriber"):
                subscriber_list = node["subscriber"]
                topic_names = ",".join(subscriber["topic_name"] for subscriber in subscriber_list)

                additional_command =f" & . /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/subscriber_node/lib/subscriber_node && ./subscriber_node --node_name {node_name} --topic_names {topic_names}"
                base_command += additional_command

            if node.get("intermediate"):
                publisher_list = node["intermediate"][0]["publisher"]
                subscriber_list = node["intermediate"][0]["subscriber"]

                topic_names_pub = ",".join(publisher["topic_name"] for publisher in publisher_list)
                payload_sizes = ",".join(str(publisher["payload_size"]) for publisher in publisher_list)
                period_ms = ",".join(str(publisher["period_ms"]) for publisher in publisher_list)
                topic_names_sub = ",".join(subscriber["topic_name"] for subscriber in subscriber_list)

                additional_command =f" & . /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/intermediate_node/lib/intermediate_node && ./intermediate_node --node_name {node_name} --topic_names_pub {topic_names_pub} --topic_names_sub {topic_names_sub} -s {payload_sizes} -p {period_ms}"
                base_command += additional_command

            # コマンドの最後には wait 命令をつける
            if index == len(nodes) - 1:
                additional_command = " & wait"
                base_command += additional_command


        additional_content = textwrap.dedent(f"""
        CMD ["/bin/bash", "-c", "{base_command}"]
        """
        )
        dockerfile_content += additional_content

        host_dir = os.path.join(output_dir, f"{host_name}")
        os.makedirs(host_dir, exist_ok=True)

        docker_file_path = os.path.join(host_dir, "Dockerfile")
        with open(docker_file_path, "w") as dockerfile:
            dockerfile.write(dockerfile_content)

    return 

# JSONファイルを受け取り、ホストの数だけ生成したDockerfileをまとめて起動するdocker-compose.ymlをルートディレクトリに生成する
#  json -> docker-compose.ymlの生成 
def generate_docker_compose(json_content):
    docker_compose_content = textwrap.dedent("services:")

    hosts = json_content["hosts"]
    for index, host_dict in enumerate(hosts):
        host_name = host_dict["host_name"]
        service_name = f"service_{host_name}"

        additional_content = textwrap.dedent(f"""
          {service_name}:
            build:
              context: Dockerfiles/{host_name}
              dockerfile: Dockerfile
            volumes:
              - ${{PWD}}/performance_test/logs:/root/performance_ws/src/graduate_research/performance_test/logs_local
            container_name: {host_name}
        """
        )
        additional_content = "  " + additional_content.replace("\n", "\n  ")


        docker_compose_content += additional_content

    docker_compose_file_path = "../compose.yml"
    with open(docker_compose_file_path, "w") as docker_compose_file:
        docker_compose_file.write(docker_compose_content)

    return


if __name__ == "__main__":
    args = sys.argv
    json_content = load_json_file(args)

    generate_dockerfiles(json_content)
    generate_docker_compose(json_content)