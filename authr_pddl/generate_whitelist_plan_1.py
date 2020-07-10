#!/usr/bin/env python

agents = ["Robot","Human"]
things = ["Toy1","Toy2","Toy3","Toy4","Battery1","Battery2","Battery3","Battery4"]

pairs = []
id_lookup = {
    "WP1:Toy1":"0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9",
    "Goal:Toy4":"0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af",
    "Goal:Battery1":"0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224",
    "WP2:Battery4":"0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff",
    "WP2:Toy2":"0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021",
    "WP1:Battery3":"0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675",
    "WP1:Toy4":"0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0",
    "Goal:Battery3":"0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111",
    "Initial:Box2":"0.3-thing-af10da1f-c8a2-4282-b33f-5be927c2e549",
    "WP1:Toy3":"0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8",
    "Initial:Battery4":"0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05",
    "WP2:Toy3":"0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5",
    "Initial:Battery1":"0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8",
    "Initial:Human":"0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26",
    "WP2:Toy4":"0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235",
    "Initial:Battery3":"0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b",
    "Initial:Toy3":"0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4",
    "Goal:Battery4":"0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af",
    "WP1:Battery4":"0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67",
    "WP2:Battery2":"0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91",
    "Initial:Toy2":"0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a",
    "WP1:Battery2":"0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f",
    "Goal:Toy2":"0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c",
    "WP1:Battery1":"0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68",
    "WP2:Battery3":"0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff",
    "Goal:Battery2":"0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e",
    "Initial:Toy1":"0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5",
    "Initial:Battery2":"0.3-thing-d879237d-900f-4740-9786-ed27e38dca06",
    "Goal:Toy1":"0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f",
    "Initial:Box1":"0.3-thing-02c8aa84-5343-48a1-beb1-d6a0d0f2b459",
    "WP1:Toy2":"0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e",
    "Initial:Robot":"0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce",
    "WP2:Battery1":"0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09",
    "Goal:Toy3":"0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654",
    "Initial:Box3":"0.3-thing-6b88713a-a8d6-407c-a4f1-f2575b78ef57",
    "Initial:Box4":"0.3-thing-16bf178c-75bf-4a28-9ee2-710a88cfcb94",
    "Initial:Toy4":"0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22",
    "WP2:Toy1":"0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305"
}

for agent in agents:
    for thing in things:
        pairs.append(("Initial:"+agent,"Initial:"+thing))
        pairs.append(("Initial:"+agent,"WP1:"+thing))
        pairs.append(("Initial:"+agent,"WP2:"+thing))
        pairs.append(("WP1:"+thing,"Initial:"+agent))
        pairs.append(("WP2:"+thing,"Initial:"+agent))

for thing in things:
    pairs.append(("WP1:"+thing,"Initial:"+thing))
    pairs.append(("Initial:"+thing,"WP1:"+thing))
    pairs.append(("WP2:"+thing,"Goal:"+thing))
    pairs.append(("Goal:"+thing,"WP2:"+thing))

for thing1 in things:
    for thing2 in things:
        pairs.append(("WP1:"+thing1,"WP2:"+thing2))
        pairs.append(("WP2:"+thing1,"WP1:"+thing2))
        pairs.append(("Goal:"+thing1,"Initial:"+thing2))

result = []

for pair in pairs:
    result.append((id_lookup[pair[0]],id_lookup[pair[1]],pair[0],pair[1]))

print("\n".join(["('{0}', '{1}'), \t# {2}->{3}".format(*pair)for pair in result]))
