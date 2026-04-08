//
// Created by Codex on 26-03-29.
//

#ifndef SCENE_GRAPH_MAP_IO_H
#define SCENE_GRAPH_MAP_IO_H

#include <string>

class SceneGraph;

class SceneGraphMapIO {
public:
    explicit SceneGraphMapIO(SceneGraph& scene_graph);

    bool save(const std::string& save_name = "");
    bool load(const std::string& save_name);

private:
    SceneGraph& scene_graph_;
};

#endif // SCENE_GRAPH_MAP_IO_H
