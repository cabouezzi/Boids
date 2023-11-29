//
//  DroneController.swift
//
//  Created by Chaniel Ezzi on 6/4/21.
//

import SceneKit

public struct BoidFlockingData {
    /// The number of other boids in perception.
    public internal(set) var flockmatesInView: Int
    /// The center of the flock of boids in perception.
    public internal(set) var flockmatesCenter: SCNVector3
    /// The average direction that boids in perception are heading.
    public internal(set) var averageFlockDirection: SCNVector3
    /// The average separation between the boids in perception.
    public internal(set) var averageSeparationDirection: SCNVector3
    
}

public class DroneController{
    open var boids: [any Boid] = []
    
    func updateDrones(in world: SCNPhysicsWorld) {
        for boid in boids {
            boid.updateDroning(world, droneController: self)
        }
    }
    
    func followTarget(_ target: SCNNode?) {
        for boid in boids {
            boid.droneTarget = target
        }
    }
    
    func setBoidSettings(_ settings: BoidSettings) {
        for boid in boids {
            boid.boidSettings = settings
        }
    }
    
    func closestBoid(to boid: Boid) -> Boid? {
        var closestDistance: Float?
        var closestBoid:( any Boid)?
        
        for other in boids where other !== boid {
            let separation = other.worldPosition.distanceTo(boid.worldPosition)
            guard separation <= boid.boidSettings.perceptionRadius else { continue }
            
            if separation < (closestDistance ?? .infinity) {
                closestDistance = separation
                closestBoid = other
            }
        }
        
        return closestBoid
    }
    
    func flockingDataForBoid(_ boid: Boid) -> BoidFlockingData {
        var flockmatesInView: Int = 0
        var flockmatesCenter = SCNVector3.zero
        var averageFlockDirection = SCNVector3.zero
        var averageSeparationDistance: Float = 0
        
        //Scan for neighboring drones
        for other in boids where other !== boid {
            guard other.worldPosition.distanceTo(boid.worldPosition) <= boid.boidSettings.perceptionRadius else { continue }
            
            flockmatesInView += 1
            flockmatesCenter += other.worldPosition
            averageFlockDirection += other.velocity
            
            if let neighbor = closestBoid(to: other) {
                averageSeparationDistance += other.worldPosition.distanceTo(neighbor.worldPosition)
            }
        }
        
        guard flockmatesInView != 0 else {
            return BoidFlockingData(flockmatesInView: 0,
                                    flockmatesCenter: boid.worldPosition,
                                    averageFlockDirection: boid.velocity,
                                    averageSeparationDirection: .zero)
        }
        
        flockmatesCenter = flockmatesCenter / Float(flockmatesInView)
        averageFlockDirection = averageFlockDirection / Float(flockmatesInView)
        averageSeparationDistance = averageSeparationDistance / Float(flockmatesInView)
        
        // Clips magnitude to minimum separation
        if averageSeparationDistance < boid.boidSettings.minSeparationDistance {
            averageSeparationDistance = boid.boidSettings.minSeparationDistance
        }
        
        // Translates the magnitude of separation to the closest path to the closest flockmate
        let translatedSeparation = (closestBoid(to: boid)!.worldPosition - boid.worldPosition).normalized() * averageSeparationDistance
        let separatedPoint = closestBoid(to: boid)!.worldPosition + translatedSeparation
        
        return BoidFlockingData(flockmatesInView: flockmatesInView,
                                 flockmatesCenter: flockmatesCenter,
                                 averageFlockDirection: averageFlockDirection,
                                 averageSeparationDirection: separatedPoint)
    }
}
