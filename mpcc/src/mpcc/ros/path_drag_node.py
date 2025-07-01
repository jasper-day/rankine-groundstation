import rclpy
from rclpy.node import Node, Service
from mpcc_interfaces.srv import ConvergePath
from mpcc.pydubins.cpp_solve import solve_path, SolverConfig
from mpcc.pydubins.dubins_solver import map_types
import toml
from pprint import pformat
from mpcc.serialization_schema import dubins2json, json2dubins

class PathDragNode(Node):
    config: dict[str, any]
    converge_path_service: Service

    def __init__(self, config: dict[str, any]):
        super().__init__("path_drag_py")
        self.config = SolverConfig(**config)

        self.converge_path_service = self.create_service(
            ConvergePath, "/gs/converge_path", callback = self.converge_path
        )

        self.get_logger().info("Started path drag node. Parameters:\n" + pformat(config))
    
    def converge_path(self, req: ConvergePath.Request, res: ConvergePath.Response):
        self.get_logger().info("Got path")
        path = req.path
        dubins = json2dubins(path)
        result = solve_path(self.config, dubins.get_params(), map_types(dubins.get_types()))
        res.converged = result.converged
        dubins.set_params(result.params)
        res.path = dubins2json(dubins)
        self.get_logger().info("Converged" if result.converged else "Did not converge")
        return res

def main(args=None):
    config_file = "solver_config.toml"
    config = toml.load(config_file)
    rclpy.init(args=args)
    node = PathDragNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()