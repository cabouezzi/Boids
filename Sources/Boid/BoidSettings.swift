//
//  DroneSettings.swift
//
//  Created by Chaniel Ezzi on 6/4/21.
//

import SceneKit

public struct BoidSettings {
    /// Minimum speed of the drone.
    public var minSpeed: CGFloat = 0.1
    /// Maximum speed of the drone.
    public var maxSpeed: CGFloat = 4
    /// The radius of the boid to be aware of other drones.
    public var perceptionRadius: CGFloat = 5
    /// Minimum separation between drones.
    public var minSeparationDistance: CGFloat = 1
    /// The maximum steering capability of the drone.
    public var maxSteerForce: CGFloat = 3
    
    /// The weight of the force that attracts the drone to a target node.
    public var targetForceWeight: CGFloat = 5
    
    /// The weight of the force to travel in the same direction as other drones.
    public var alignmentForceWeight: CGFloat = 2
    /// The weight of the force that brings the boid closer to other drones.
    public var cohesionForceWeight: CGFloat = 1
    /// The weight of the force to keep a separation CGFloat between other drones.
    public var separationForceWeight: CGFloat = 5
    
    /// The weight of the force to avoid obstacles.
    public var avoidanceForceWeight: CGFloat = 50
    /// The distance to which the drone recognizes obstacles.
    public var avoidDistance: CGFloat = 1
    
    /// An array of the drone's percieved directions in a spherical radius with a magnitude of 1.
    private(set) var directions: [(asVector: SCNVector3, asEulerAngles: SCNVector3)] = []
    
    /// - Parameter raycount: The number of rays created in the boids field of view to detect collisions.
    public init(_ rayCount: Int = 100) {
        setDirections(rayCount)
    }
    
    /// Sets equidistant directions in a spherical radius with a magnitude of one.
    public mutating func setDirections (_ directionCount: Int) {
        directions.removeAll()
        
        let goldenRatio: Float = (1 + sqrtf(5)) / 2
        let angleIncrement: Float = (2 * .pi) * goldenRatio
        
        for i in 0..<directionCount {
            // Ratio for spreading
            let ratio = Float(i) / Float(directionCount)
            
            // Angles
            let polarAngle: Float = (acosf(1 - 2 * ratio)).truncatingRemainder(dividingBy: .pi)
            let azimuthalAngle: Float = (Float(angleIncrement) * Float(i)).truncatingRemainder(dividingBy: 2 * .pi)
            
            // Cartesian
            let x = sinf(polarAngle) * cosf(azimuthalAngle)
            let y = cosf(polarAngle)
            let z = sinf(polarAngle) * sinf(azimuthalAngle)
            
            // Store as vector/angular options
            let direction = SCNVector3(0, azimuthalAngle, polarAngle)
            let vector = SCNVector3(x, y, z)
            
            directions.append((asVector: vector, asEulerAngles: direction))
        }
    }
}

