#!/usr/bin/env python

agents = ["Robot","Human"]
things = ["Component1","Component2"]

pairs = []
id_lookup = {
    "WP1:Old Component2"	:"0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a",
    "Initial:Robot"	:"0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa",
    "WP1:New Component2"	:"0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549",
    "Initial:New Component2"	:"0.3-thing-364a7ea3-36fb-4eea-a654-1eaff059c0f7",
    "Goal:Old Component2"	:"0.3-destination-7d70a1ed-4043-47be-8803-8948c0d3e3d4",
    "Initial:Old Component2"	:"0.3-thing-169c5ff9-271f-4cd9-8e61-05cae9362815",
    "WP2:Old Component2"	:"0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8",
    "WP1:New Component1"	:"0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a",
    "Initial:Old Component1"	:"0.3-thing-b2b860bb-f37a-4e77-a84e-e2fbc1153509",
    "Goal:Old Component1"	:"0.3-destination-68fca638-95c3-4090-be0b-41828dc18122",
    "Initial:Human"	:"0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb",
    "Initial:New Component1"	:"0.3-thing-22cdc687-ef41-4c35-9b0e-f2af8b75eec9",
    "WP2:Old Component1"	:"0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb",
    "WP1:Old Component1"	:"0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce"
}

for agent in agents:
    for thing in things:
        pairs.append(("Initial:"+agent,"WP1:Old "+thing))
        pairs.append(("Initial:"+agent,"WP1:New "+thing))
        pairs.append(("Initial:"+agent,"WP2:Old "+thing))
        pairs.append(("WP1:Old "+thing,"Initial:"+agent))
        pairs.append(("WP1:New "+thing,"Initial:"+agent))
        pairs.append(("WP2:Old "+thing,"Initial:"+agent))

for thing in things:
    pairs.append(("WP1:Old "+thing,"Initial:Old "+thing))
    pairs.append(("WP1:New "+thing,"Initial:New "+thing))
    pairs.append(("Initial:Old "+thing,"WP1:Old "+thing))
    pairs.append(("Initial:New "+thing,"WP1:New "+thing))
    pairs.append(("WP2:Old "+thing,"Goal:Old "+thing))
    pairs.append(("Goal:Old "+thing,"WP2:Old "+thing))

for thing1 in things:
    for thing2 in things:
        pairs.append(("WP1:Old "+thing1,"WP2:Old "+thing2))
        pairs.append(("WP1:New "+thing1,"WP2:Old "+thing2))
        pairs.append(("WP1:New "+thing1,"WP1:Old "+thing2))
        pairs.append(("WP1:Old "+thing1,"WP1:New "+thing2))
        pairs.append(("WP2:Old "+thing1,"WP1:New "+thing2))
        pairs.append(("WP2:Old "+thing1,"WP1:Old "+thing2))

result = []

for pair in pairs:
    result.append((id_lookup[pair[0]],id_lookup[pair[1]],pair[0],pair[1]))

print("\n".join(["('{0}', '{1}'), \t# {2}->{3}".format(*pair)for pair in result]))
