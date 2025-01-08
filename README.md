# graduate_research

以下のパッケージを組み合わせます:
1. ノード名やトピック名、ペイロードサイズ等のoptionを制御できるPublisher, Subscriberを作成するパッケージ
2. jsonファイルをparseして、立ち上げるコンテナ、ノードの種類、ノードの設定を書き込んだdocker-compose.ymlを作成するパッケージ

## Build
```bash
colcon build
```

## Run
例えば、publisher_nodeを立ち上げたい場合
``` bash
source install/setup.bash
cd install/publisher_node/lib/publisher_node
./publisher_node --node_name my_node --topic_name sample -s 64 -p 20
```

## Docker compose
まずはPythonプロジェクトを用いて、JSONファイル(のパス)からDockerfileとdocker-compose.ymlを生成
```bash
cd parse_json_to_dockerfiles
python3 parse_json.py ../examples/topology_example/topology_example.json 
```
生成したdocker-compose.ymlからコンテナイメージを生成し、実行する。別のdocker-compose.ymlを実行していた場合は、`docker-compose down`を叩いておく
```bash
docker-compose build --no-cache
docker-compose up
```
