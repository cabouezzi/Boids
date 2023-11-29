//
//  DroneSettings.swift
//
//  Created by Chaniel Ezzi on 6/4/21.
//

import SceneKit

public struct BoidSettings {
    /// Minimum speed of the drone.
    public var minSpeed: BFloat = 0.1
    /// Maximum speed of the drone.
    public var maxSpeed: BFloat = 4
    /// The radius of the boid to be aware of other drones.
    public var perceptionRadius: BFloat = 5
    /// Minimum separation between drones.
    public var minSeparationDistance: BFloat = 1
    /// The maximum steering capability of the drone.
    public var maxSteerForce: BFloat = 3
    
    /// The weight of the force that attracts the drone to a target node.
    public var targetForceWeight: BFloat = 5
    
    /// The weight of the force to travel in the same direction as other drones.
    public var alignmentForceWeight: BFloat = 2
    /// The weight of the force that brings the boid closer to other drones.
    public var cohesionForceWeight: BFloat = 1
    /// The weight of the force to keep a separation BFloat between other drones.
    public var separationForceWeight: BFloat = 5
    
    /// The weight of the force to avoid obstacles.
    public var avoidanceForceWeight: BFloat = 50
    /// The distance to which the drone recognizes obstacles.
    public var avoidDistance: BFloat = 1
    
    /// An array of the drone's percieved directions in a spherical radius with a magnitude of 1.
    private(set) var directions: [(asVector: SCNVector3, asEulerAngles: SCNVector3)] = []
    
    /// - Parameter raycount: The number of rays created in the boids field of view to detect collisions.
    public init(_ rayCount: Int = 100) {
        setDirections(rayCount)
    }
    
    /// Sets equidistant directions in a spherical radius with a magnitude of one.
    public mutating func setDirections (_ directionCount: Int) {
        directions.removeAll()
        
        let goldenRatio: BFloat = (1 + sqrt(5)) / 2
        let angleIncrement: BFloat = (2 * .pi) * goldenRatio
        
        for i in 0..<directionCount {
            // Ratio for spreading
            let ratio = BFloat(i) / BFloat(directionCount)
            
            // Angles
            let polarAngle: BFloat = (acos(1 - 2 * ratio)).truncatingRemainder(dividingBy: .pi)
            let azimuthalAngle: BFloat = (BFloat(angleIncrement) * BFloat(i)).truncatingRemainder(dividingBy: 2 * .pi)
            
            // Cartesian
            let x = sin(polarAngle) * cos(azimuthalAngle)
            let y = cos(polarAngle)
            let z = sin(polarAngle) * sin(azimuthalAngle)
            
            // Store as vector/angular options
            let direction = SCNVector3(0, azimuthalAngle, polarAngle)
            let vector = SCNVector3(x, y, z)
            
            directions.append((asVector: vector, asEulerAngles: direction))
        }
    }
}

