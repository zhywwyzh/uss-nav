# Scene Graph Benchmark Runner

This tool is a sidecar benchmark runner. It does not modify the native `SceneGraph -> /scene_graph/prompt -> LLM -> /scene_graph/llm_ans` loop.

It uses:
- `/scene_graph/vis`
- `/object_all_vis`
- optional `/odom_world`

to build a compact snapshot, synthesize a benchmark prompt, call an OpenAI-compatible model, and record:
- input token count
- output token count
- TTFT
- end-to-end latency

## Topics

The runner publishes:
- `/scene_graph/benchmark_prompt`
- `/scene_graph/benchmark_llm_ans`
- `/scene_graph/benchmark_metrics`

## Example

```bash
python3 ros_ws/uss-nav/ws_main/src/planner/scene_graph/scripts/scene_graph_benchmark_runner.py \
  _task_type:=expl_prediction \
  _target_text:="Find shelf full of storage boxes" \
  _llm_base_url:="http://127.0.0.1:2231/v1" \
  _llm_api_key:="EMPTY" \
  _llm_model_name:="" \
  _output_path:="/tmp/scene_graph_benchmark.jsonl"
```

## Supported task types

- `expl_prediction`
- `room_prediction`
- `terminate_obj_id`
- `df_demo`

## Notes

- `expl_prediction` can run with only `/scene_graph/vis`.
- `room_prediction`, `terminate_obj_id`, and `df_demo` expect `/object_all_vis`.
- TTFT is only available when the upstream LLM endpoint supports streaming.
