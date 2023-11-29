//
//  SCNVector+Math.swift
//  
//
//  Created by Chaniel Ezzi on 11/28/23.
//

import SceneKit

internal extension SCNVector3 {
    static func + (lhs: SCNVector3, rhs: SCNVector3) -> SCNVector3 {
        let x = lhs.x + rhs.x
        let y = lhs.y + rhs.y
        let z = lhs.z + rhs.z
        return SCNVector3(x, y, z)
    }
    
    static func - (lhs: SCNVector3, rhs: SCNVector3) -> SCNVector3 {
        let x = lhs.x - rhs.x
        let y = lhs.y - rhs.y
        let z = lhs.z - rhs.z
        return SCNVector3(x, y, z)
    }
    
    static func += (lhs: inout SCNVector3, rhs: SCNVector3) {
        lhs.x += rhs.x
        lhs.y += rhs.y
        lhs.z += rhs.z
    }
    
    static func -= (lhs: inout SCNVector3, rhs: SCNVector3) {
        lhs.x -= rhs.x
        lhs.y -= rhs.y
        lhs.z -= rhs.z
    }
    
    static func * (lhs: SCNVector3, rhs: BFloat) -> SCNVector3 {
        return SCNVector3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs)
    }
    
    static func / (lhs: SCNVector3, rhs: BFloat) -> SCNVector3 {
        return SCNVector3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs)
    }
    
    static func *= (lhs: inout SCNVector3, rhs: BFloat) {
        lhs.x *= rhs
        lhs.y *= rhs
        lhs.z *= rhs
    }
    static func /= (lhs: inout SCNVector3, rhs: BFloat) {
        lhs.x /= rhs
        lhs.y /= rhs
        lhs.z /= rhs
    }
    
    static func == (lhs: SCNVector3, rhs: SCNVector3) -> Bool {
        guard lhs.x == rhs.x else { return false }
        guard lhs.y == rhs.y else { return false }
        guard lhs.z == rhs.z else { return false }
        return true
    }
    
    static func != (lhs: SCNVector3, rhs: SCNVector3) -> Bool {
        return !(lhs == rhs)
    }
    
    static var zero: SCNVector3 {
        return SCNVector3(0, 0, 0)
    }
    
    func distanceTo (_ otherVector: SCNVector3) -> BFloat {
        let xDistance = abs(otherVector.x - x)
        let yDistance = abs(otherVector.y - y)
        let zDistance = abs(otherVector.z - z)
        
        return sqrt(pow(xDistance, 2) + pow(yDistance, 2) + pow(zDistance, 2))
    }
    
    func magnitude () -> BFloat {
        if self == SCNVector3.zero { return 0 }
        return distanceTo(SCNVector3.zero)
    }
    
    /// The given vector scaled to a magnitude of 1.
    func normalized () -> SCNVector3 {
        if self == SCNVector3.zero { return self }
        if magnitude().sign == .minus { fatalError() }
        return self / magnitude()
    }
    
    /// Scales the vector to the given magnitude
    mutating func setMagnitude (_ magnitude: BFloat) {
        self = normalized() * magnitude
    }
}
