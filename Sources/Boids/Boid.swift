//
//  Droning.swift
//
//  Created by Chaniel Ezzi on 6/4/21.
//

#if os(macOS)
public typealias BFloat = CGFloat
#else
public typealias BFloat = Float
#endif

import SceneKit

public protocol Boid: SCNNode {
    var velocity: SCNVector3 { get set }
    var boidSettings: BoidSettings { get set }
    var droneTarget: SCNNode? { get set }
}

open class BoidPrototype: SCNNode, Boid {
    final public var velocity = SCNVector3(x: 0, y: 0, z: 0)
    open var boidSettings = BoidSettings()
    open var droneTarget: SCNNode?
}

extension Boid {
    func updateDroning (_ world: SCNPhysicsWorld, droneController: BoidController?) {
        var acceleration = SCNVector3(x: 0, y: 0, z: 0)
        
        if let magnet = droneTarget {
            let direction = magnet.worldPosition - worldPosition
            let magnetForce = scaledForce(direction) * boidSettings.targetForceWeight

            acceleration += magnetForce
        }

        if let controller = droneController {
            let data = controller.flockingDataForBoid(self)

            let directionToFlock = data.flockmatesCenter - worldPosition

            let alignmentForce = scaledForce(data.averageFlockDirection) * boidSettings.alignmentForceWeight
            let cohesionForce = scaledForce(directionToFlock) * boidSettings.cohesionForceWeight
            let separationForce = scaledForce(data.averageSeparationDirection) * boidSettings.separationForceWeight

            //TODO: apply/don't apply settings
            acceleration += alignmentForce
            acceleration += cohesionForce
            acceleration -= separationForce
        }
        
        if isObstructed(world) {
            let avoidanceVector = unobstructedDirection(world)
            let avoidanceForce = scaledForce(avoidanceVector) * boidSettings.avoidanceForceWeight
            acceleration = avoidanceForce
        }
        
        if acceleration.magnitude() > boidSettings.maxSteerForce {
            acceleration.setMagnitude(boidSettings.maxSteerForce)
        }
        
        velocity += acceleration * BFloat(world.timeStep)
        
        //Clip to minimum speed
        if velocity.magnitude() < boidSettings.minSpeed {
            velocity.setMagnitude(boidSettings.minSpeed)
        }
        //Clip to max speed
        if velocity.magnitude() > boidSettings.maxSpeed {
            velocity.setMagnitude(boidSettings.maxSpeed)
        }
        
        updateRotation()
        worldPosition += velocity * BFloat(world.timeStep)
    }
    
    ///Updates the rotation of the node. Default is to face the +Z direction of the velocity.
    private func updateRotation () {
        let dir = worldPosition + velocity
        look(at: dir)
    }
    
    private func isObstructed(_ world: SCNPhysicsWorld) -> Bool {
        let results = world.rayTestWithSegment(from: worldPosition, to: worldPosition + velocity * boidSettings.avoidDistance, options: [.backfaceCulling : false])
        
        // Hits something that's not a boid
        if results.contains(where: { !($0.node is Self) }) {
            return true
        }
        else {
            return false
        }
    }
    
    private func distanceToCollision(_ world: SCNPhysicsWorld) -> BFloat? {
        let results = world.rayTestWithSegment(from: worldPosition, to: worldPosition + velocity * boidSettings.avoidDistance, options: [.backfaceCulling: true])
        
        // Hits something that's not a boid
        if let result = results.first(where: { !($0.node is Self) }) {
            return result.worldCoordinates.distanceTo(worldPosition)
        }
        
        return nil
    }
    
    private func unobstructedDirection(_ world: SCNPhysicsWorld) -> SCNVector3 {
        var mostPromising = velocity
        var longest: BFloat = 0
        var best: SCNVector3?
        
        for direction in boidSettings.directions {
            let results = world.rayTestWithSegment(from: worldPosition, to: worldPosition + (velocity + direction.asVector) * boidSettings.avoidDistance, options: [.backfaceCulling : false, .collisionBitMask : 1])
            // Hits something that's not a boid
            if let result = results.first(where: { !($0.node is Self) }) {
                let point = result.worldCoordinates
                if worldPosition.distanceTo(point) > longest {
                    mostPromising = direction.asVector
                    longest = point.distanceTo(worldPosition)
                }
            }
            // Doesn't hit anything
            else if best == nil || direction.asVector.distanceTo(velocity) < best!.distanceTo(velocity) {
                best = direction.asVector
            }
        }
        
        return best ?? mostPromising
    }
    
    private func scaledForce(_ vector: SCNVector3) -> SCNVector3 {
        var v = vector.normalized() * boidSettings.maxSpeed - velocity
        if v.magnitude() > boidSettings.maxSteerForce {
            v.setMagnitude(boidSettings.maxSteerForce)
        }
        return v
    }
}
