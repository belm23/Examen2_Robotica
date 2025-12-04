#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor
import numpy as np

class DQNTestNode(Node):
    """Test trained DQN agent"""
    
    def __init__(self, model_path: str):
        super().__init__('dqn_test_node')
        
        self.state_size = 12
        self.action_size = 5
        
        # Load trained agent
        self.agent = DQNAgent(self.state_size, self.action_size)
        self.agent.load(model_path)
        self.agent.epsilon = 0.0  # Greedy policy only
        
        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=10)
        
        self.get_logger().info(f"Loaded model from {model_path}")
    
    def get_processed_state(self):
        self.get_logger().info(f"LiDAR data size: {len(self.env.scan_data)}")
        return self.state_processor.get_state(
            self.env.scan_data,
            self.env.position,
            self.env.goal_position,
            self.env.yaw
        )
    
    def test(self, n_episodes: int = 10):
        """Run test episodes"""
        
        successes = 0
        total_rewards = []
        
        for episode in range(n_episodes):
            self.env.reset(random_goal=True)
            rclpy.spin_once(self.env, timeout_sec=0.5)
            
            # 2. ESPERA CRÍTICA DE INICIALIZACIÓN DE DATOS (CORRECCIÓN)
            # Asegura que self.env.scan_data no sea None antes de procesar
            self.get_logger().info(f"Episode {episode+1}/{n_episodes}: Waiting for LiDAR data...")
            
            # Asumiendo que self.env.scan_data es el mensaje LaserScan que es None inicialmente
            while self.env.scan_data is None and rclpy.ok():
                rclpy.spin_once(self.env, timeout_sec=0.1)

            # Si salimos del bucle pero el dato sigue siendo None (ej. ROS cerró), saltar episodio
            if self.env.scan_data is None:
                self.get_logger().error("Failed to receive initial LiDAR data. Skipping episode.")
                continue
                
            # 3. Iniciar el episodio
            state = self.get_processed_state()
            episode_reward = 0
            
            for step in range(500):
                action = self.agent.act(state, training=False)
                # La función step() debería incluir una llamada a rclpy.spin_once()
                # para actualizar el estado del LiDAR antes de que se obtenga el siguiente estado.
                _, reward, done = self.env.step(action)
                state = self.get_processed_state()
                
                episode_reward += reward
                
                if done:
                    if self.env.is_goal_reached():
                        successes += 1
                        self.get_logger().info(
                            f"Episode {episode+1}: SUCCESS! "
                            f"Steps: {step+1}, Reward: {episode_reward:.2f}"
                        )
                    else:
                        self.get_logger().info(
                            f"Episode {episode+1}: COLLISION. "
                            f"Steps: {step+1}, Reward: {episode_reward:.2f}"
                        )
                    break
                
                # Este spin_once ya estaba, es bueno para mantener la comunicación
                rclpy.spin_once(self.env, timeout_sec=0.01)
            
            total_rewards.append(episode_reward)
        
        # Print statistics
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f"Test Results over {n_episodes} episodes:")
        self.get_logger().info(f"Success Rate: {successes/n_episodes*100:.1f}%")
        self.get_logger().info(f"Avg Reward: {np.mean(total_rewards):.2f}")
        self.get_logger().info(f"Std Reward: {np.std(total_rewards):.2f}")
        self.get_logger().info("="*50)

def main(args=None):
    import sys
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run dqn_robot_nav test_node <model_path>")
        return
    
    model_path = sys.argv[1]
    tester = DQNTestNode(model_path)
    
    try:
        tester.test(n_episodes=10)
    except KeyboardInterrupt:
        pass
    finally:
        tester.env.send_velocity(0.0, 0.0)
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()