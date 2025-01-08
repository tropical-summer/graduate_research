"""
JSONファイルを受け取ったら、ホストの数のDockerfileと、それらをまとめて立ち上げるdocker-compose.ymlを作成する
入力例: python parse_json.py examples/topology_example/topology_example.json
"""

import sys
import json
import os
import textwrap

# コマンドラインからJSONファイルのパスを受け取り、そのJSONファイルを取得する
# < (args) -> json > 
def load_json_file(args):
    file_path = args[1] # args[0]には実行ファイル名、args[1]にコマンドライン引数が来る
    with open(file_path, "r") as f:
        json_content = json.load(f)

    return json_content


# JSONファイルを受けとり、各ホストに対応するDockerfileを生成する.生成したDockerfileの数だけディレクトリを作り、Dockerfileはその下に置く
# < json -> list[Dockerfile]の生成 >
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
                    topic_name = node["publisher"][0]["topic_name"]
                    payload_size = node["publisher"][0]["payload_size"]
                    period_ms = node["publisher"][0]["period_ms"]

                    additional_command = f". /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node && ./publisher_node --node_name {node_name} --topic_name {topic_name} -s {payload_size} -p {period_ms}"
                    base_command += additional_command

                if node.get("subscriber"):
                    topic_name = node["subscriber"][0]["topic_name"]

                    additional_command =f". /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/subscriber_node/lib/subscriber_node && ./subscriber_node --node_name {node_name} --topic_name {topic_name}"
                    base_command += additional_command

                continue

            # 複数のノードをバックグラウンドで同時に起動するため、通常は & で結ぶ
            if node.get("publisher"):
                topic_name = node["publisher"][0]["topic_name"]
                payload_size = node["publisher"][0]["payload_size"]
                period_ms = node["publisher"][0]["period_ms"]

                additional_command = f" & . /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node && ./publisher_node --node_name {node_name} --topic_name {topic_name} -s {payload_size} -p {period_ms}"
                base_command += additional_command

            if node.get("subscriber"):
                topic_name = node["subscriber"][0]["topic_name"]

                additional_command =f" & . /root/performance_ws/src/graduate_research/install/setup.sh && cd /root/performance_ws/src/graduate_research/install/subscriber_node/lib/subscriber_node && ./subscriber_node --node_name {node_name} --topic_name {topic_name}"
                base_command += additional_command

            # コマンドの最後には wait 命令をつける
            if index == len(nodes)-1:
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
# < json -> docker-compose.ymlの生成 >
def generate_docker_compose(json_content):
    docker_compose_content = textwrap.dedent(f"""
    version: '3'
    services:
    """
    )

    hosts = json_content["hosts"]
    for index, host_dict in enumerate(hosts):
        host_name = host_dict["host_name"]
        service_name = f"service_{host_name}"

        additional_content = textwrap.dedent(f"""
          {service_name}:
            build:
              context: Dockerfiles/{host_name}
              dockerfile: Dockerfile
            container_name: {host_name}
        """
        )
        additional_content = "  " + additional_content.replace("\n", "\n  ")


        docker_compose_content += additional_content

    docker_compose_file_path = "../docker-compose.yml"
    with open(docker_compose_file_path, "w") as docker_compose_file:
        docker_compose_file.write(docker_compose_content)

    return


if __name__ == "__main__":
    args = sys.argv
    json_content = load_json_file(args)

    generate_dockerfiles(json_content)
    generate_docker_compose(json_content)