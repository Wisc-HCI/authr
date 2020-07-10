

transport_wl = [

#===================================================================================================================================
#   Plan 1
#===================================================================================================================================

('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Initial:Robot->Initial:Toy1
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# Initial:Robot->WP1:Toy1
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# Initial:Robot->WP2:Toy1
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Toy1->Initial:Robot
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Toy1->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Initial:Robot->Initial:Toy2
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# Initial:Robot->WP1:Toy2
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# Initial:Robot->WP2:Toy2
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Toy2->Initial:Robot
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Toy2->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Initial:Robot->Initial:Toy3
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# Initial:Robot->WP1:Toy3
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# Initial:Robot->WP2:Toy3
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Toy3->Initial:Robot
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Toy3->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Initial:Robot->Initial:Toy4
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# Initial:Robot->WP1:Toy4
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# Initial:Robot->WP2:Toy4
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Toy4->Initial:Robot
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Toy4->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Initial:Robot->Initial:Battery1
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# Initial:Robot->WP1:Battery1
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# Initial:Robot->WP2:Battery1
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Battery1->Initial:Robot
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Battery1->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Initial:Robot->Initial:Battery2
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# Initial:Robot->WP1:Battery2
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# Initial:Robot->WP2:Battery2
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Battery2->Initial:Robot
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Battery2->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Initial:Robot->Initial:Battery3
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# Initial:Robot->WP1:Battery3
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# Initial:Robot->WP2:Battery3
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Battery3->Initial:Robot
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Battery3->Initial:Robot
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Initial:Robot->Initial:Battery4
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# Initial:Robot->WP1:Battery4
('0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# Initial:Robot->WP2:Battery4
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP1:Battery4->Initial:Robot
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-agent-bb69c6cb-4a96-47e9-9650-be59cd5d97ce'), 	# WP2:Battery4->Initial:Robot
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Initial:Human->Initial:Toy1
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# Initial:Human->WP1:Toy1
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# Initial:Human->WP2:Toy1
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Toy1->Initial:Human
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Toy1->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Initial:Human->Initial:Toy2
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# Initial:Human->WP1:Toy2
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# Initial:Human->WP2:Toy2
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Toy2->Initial:Human
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Toy2->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Initial:Human->Initial:Toy3
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# Initial:Human->WP1:Toy3
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# Initial:Human->WP2:Toy3
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Toy3->Initial:Human
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Toy3->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Initial:Human->Initial:Toy4
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# Initial:Human->WP1:Toy4
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# Initial:Human->WP2:Toy4
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Toy4->Initial:Human
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Toy4->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Initial:Human->Initial:Battery1
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# Initial:Human->WP1:Battery1
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# Initial:Human->WP2:Battery1
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Battery1->Initial:Human
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Battery1->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Initial:Human->Initial:Battery2
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# Initial:Human->WP1:Battery2
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# Initial:Human->WP2:Battery2
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Battery2->Initial:Human
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Battery2->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Initial:Human->Initial:Battery3
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# Initial:Human->WP1:Battery3
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# Initial:Human->WP2:Battery3
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Battery3->Initial:Human
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Battery3->Initial:Human
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Initial:Human->Initial:Battery4
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# Initial:Human->WP1:Battery4
('0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# Initial:Human->WP2:Battery4
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP1:Battery4->Initial:Human
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-agent-66a30f87-c5b8-4777-9690-776439bd7f26'), 	# WP2:Battery4->Initial:Human
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# WP1:Toy1->Initial:Toy1
('0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# Initial:Toy1->WP1:Toy1
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f'), 	# WP2:Toy1->Goal:Toy1
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# Goal:Toy1->WP2:Toy1
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# WP1:Toy2->Initial:Toy2
('0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# Initial:Toy2->WP1:Toy2
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c'), 	# WP2:Toy2->Goal:Toy2
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# Goal:Toy2->WP2:Toy2
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# WP1:Toy3->Initial:Toy3
('0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# Initial:Toy3->WP1:Toy3
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654'), 	# WP2:Toy3->Goal:Toy3
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# Goal:Toy3->WP2:Toy3
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# WP1:Toy4->Initial:Toy4
('0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# Initial:Toy4->WP1:Toy4
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af'), 	# WP2:Toy4->Goal:Toy4
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# Goal:Toy4->WP2:Toy4
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# WP1:Battery1->Initial:Battery1
('0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# Initial:Battery1->WP1:Battery1
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224'), 	# WP2:Battery1->Goal:Battery1
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# Goal:Battery1->WP2:Battery1
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# WP1:Battery2->Initial:Battery2
('0.3-thing-d879237d-900f-4740-9786-ed27e38dca06', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# Initial:Battery2->WP1:Battery2
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e'), 	# WP2:Battery2->Goal:Battery2
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# Goal:Battery2->WP2:Battery2
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# WP1:Battery3->Initial:Battery3
('0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# Initial:Battery3->WP1:Battery3
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111'), 	# WP2:Battery3->Goal:Battery3
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# Goal:Battery3->WP2:Battery3
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# WP1:Battery4->Initial:Battery4
('0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# Initial:Battery4->WP1:Battery4
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af'), 	# WP2:Battery4->Goal:Battery4
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# Goal:Battery4->WP2:Battery4
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Toy1->WP2:Toy1
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Toy1->WP1:Toy1
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Toy1->Initial:Toy1
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Toy1->WP2:Toy2
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Toy1->WP1:Toy2
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Toy1->Initial:Toy2
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Toy1->WP2:Toy3
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Toy1->WP1:Toy3
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Toy1->Initial:Toy3
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Toy1->WP2:Toy4
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Toy1->WP1:Toy4
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Toy1->Initial:Toy4
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Toy1->WP2:Battery1
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Toy1->WP1:Battery1
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Toy1->Initial:Battery1
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Toy1->WP2:Battery2
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Toy1->WP1:Battery2
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Toy1->Initial:Battery2
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Toy1->WP2:Battery3
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Toy1->WP1:Battery3
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Toy1->Initial:Battery3
('0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Toy1->WP2:Battery4
('0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Toy1->WP1:Battery4
('0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Toy1->Initial:Battery4
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Toy2->WP2:Toy1
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Toy2->WP1:Toy1
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Toy2->Initial:Toy1
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Toy2->WP2:Toy2
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Toy2->WP1:Toy2
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Toy2->Initial:Toy2
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Toy2->WP2:Toy3
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Toy2->WP1:Toy3
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Toy2->Initial:Toy3
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Toy2->WP2:Toy4
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Toy2->WP1:Toy4
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Toy2->Initial:Toy4
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Toy2->WP2:Battery1
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Toy2->WP1:Battery1
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Toy2->Initial:Battery1
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Toy2->WP2:Battery2
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Toy2->WP1:Battery2
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Toy2->Initial:Battery2
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Toy2->WP2:Battery3
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Toy2->WP1:Battery3
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Toy2->Initial:Battery3
('0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Toy2->WP2:Battery4
('0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Toy2->WP1:Battery4
('0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Toy2->Initial:Battery4
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Toy3->WP2:Toy1
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Toy3->WP1:Toy1
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Toy3->Initial:Toy1
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Toy3->WP2:Toy2
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Toy3->WP1:Toy2
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Toy3->Initial:Toy2
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Toy3->WP2:Toy3
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Toy3->WP1:Toy3
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Toy3->Initial:Toy3
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Toy3->WP2:Toy4
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Toy3->WP1:Toy4
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Toy3->Initial:Toy4
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Toy3->WP2:Battery1
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Toy3->WP1:Battery1
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Toy3->Initial:Battery1
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Toy3->WP2:Battery2
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Toy3->WP1:Battery2
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Toy3->Initial:Battery2
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Toy3->WP2:Battery3
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Toy3->WP1:Battery3
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Toy3->Initial:Battery3
('0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Toy3->WP2:Battery4
('0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Toy3->WP1:Battery4
('0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Toy3->Initial:Battery4
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Toy4->WP2:Toy1
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Toy4->WP1:Toy1
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Toy4->Initial:Toy1
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Toy4->WP2:Toy2
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Toy4->WP1:Toy2
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Toy4->Initial:Toy2
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Toy4->WP2:Toy3
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Toy4->WP1:Toy3
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Toy4->Initial:Toy3
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Toy4->WP2:Toy4
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Toy4->WP1:Toy4
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Toy4->Initial:Toy4
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Toy4->WP2:Battery1
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Toy4->WP1:Battery1
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Toy4->Initial:Battery1
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Toy4->WP2:Battery2
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Toy4->WP1:Battery2
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Toy4->Initial:Battery2
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Toy4->WP2:Battery3
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Toy4->WP1:Battery3
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Toy4->Initial:Battery3
('0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Toy4->WP2:Battery4
('0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Toy4->WP1:Battery4
('0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Toy4->Initial:Battery4
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Battery1->WP2:Toy1
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Battery1->WP1:Toy1
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Battery1->Initial:Toy1
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Battery1->WP2:Toy2
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Battery1->WP1:Toy2
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Battery1->Initial:Toy2
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Battery1->WP2:Toy3
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Battery1->WP1:Toy3
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Battery1->Initial:Toy3
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Battery1->WP2:Toy4
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Battery1->WP1:Toy4
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Battery1->Initial:Toy4
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Battery1->WP2:Battery1
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Battery1->WP1:Battery1
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Battery1->Initial:Battery1
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Battery1->WP2:Battery2
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Battery1->WP1:Battery2
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Battery1->Initial:Battery2
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Battery1->WP2:Battery3
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Battery1->WP1:Battery3
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Battery1->Initial:Battery3
('0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Battery1->WP2:Battery4
('0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Battery1->WP1:Battery4
('0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Battery1->Initial:Battery4
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Battery2->WP2:Toy1
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Battery2->WP1:Toy1
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Battery2->Initial:Toy1
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Battery2->WP2:Toy2
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Battery2->WP1:Toy2
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Battery2->Initial:Toy2
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Battery2->WP2:Toy3
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Battery2->WP1:Toy3
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Battery2->Initial:Toy3
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Battery2->WP2:Toy4
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Battery2->WP1:Toy4
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Battery2->Initial:Toy4
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Battery2->WP2:Battery1
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Battery2->WP1:Battery1
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Battery2->Initial:Battery1
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Battery2->WP2:Battery2
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Battery2->WP1:Battery2
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Battery2->Initial:Battery2
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Battery2->WP2:Battery3
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Battery2->WP1:Battery3
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Battery2->Initial:Battery3
('0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Battery2->WP2:Battery4
('0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Battery2->WP1:Battery4
('0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Battery2->Initial:Battery4
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Battery3->WP2:Toy1
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Battery3->WP1:Toy1
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Battery3->Initial:Toy1
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Battery3->WP2:Toy2
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Battery3->WP1:Toy2
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Battery3->Initial:Toy2
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Battery3->WP2:Toy3
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Battery3->WP1:Toy3
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Battery3->Initial:Toy3
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Battery3->WP2:Toy4
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Battery3->WP1:Toy4
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Battery3->Initial:Toy4
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Battery3->WP2:Battery1
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Battery3->WP1:Battery1
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Battery3->Initial:Battery1
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Battery3->WP2:Battery2
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Battery3->WP1:Battery2
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Battery3->Initial:Battery2
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Battery3->WP2:Battery3
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Battery3->WP1:Battery3
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Battery3->Initial:Battery3
('0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Battery3->WP2:Battery4
('0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Battery3->WP1:Battery4
('0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Battery3->Initial:Battery4
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-8a11a4ea-4912-435b-87b9-1258db06d305'), 	# WP1:Battery4->WP2:Toy1
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-9ef7443a-4a77-4ced-9c97-f276dc66b2e9'), 	# WP2:Battery4->WP1:Toy1
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5'), 	# Goal:Battery4->Initial:Toy1
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-70a10821-f87f-443d-899a-b10bc8a78021'), 	# WP1:Battery4->WP2:Toy2
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-c8a0543f-3415-4b31-8af6-711ae1853c6e'), 	# WP2:Battery4->WP1:Toy2
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a'), 	# Goal:Battery4->Initial:Toy2
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-84a522b7-aef7-44bd-b2ab-4af6c07ff0b5'), 	# WP1:Battery4->WP2:Toy3
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-250bbaea-50af-40ed-adec-8d0c008fc9a8'), 	# WP2:Battery4->WP1:Toy3
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4'), 	# Goal:Battery4->Initial:Toy3
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-42643189-5f76-47e9-8e36-bd1ad409b235'), 	# WP1:Battery4->WP2:Toy4
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-709cdc0c-e4c3-401e-91d9-cb02fd9243a0'), 	# WP2:Battery4->WP1:Toy4
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22'), 	# Goal:Battery4->Initial:Toy4
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-9325ac02-e134-4313-9357-cbd4d6688b09'), 	# WP1:Battery4->WP2:Battery1
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-cf960ff8-6010-46c3-b660-6903af267d68'), 	# WP2:Battery4->WP1:Battery1
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8'), 	# Goal:Battery4->Initial:Battery1
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-d21c998d-7bee-4664-92cc-a6db2e7e6e91'), 	# WP1:Battery4->WP2:Battery2
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-de7e979b-9d87-40e2-bd15-0a572be0415f'), 	# WP2:Battery4->WP1:Battery2
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-d879237d-900f-4740-9786-ed27e38dca06'), 	# Goal:Battery4->Initial:Battery2
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-a833feb0-7ee3-4d3b-8ac0-001c9a775cff'), 	# WP1:Battery4->WP2:Battery3
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-76a50b72-083f-4798-a32a-b7be6faff675'), 	# WP2:Battery4->WP1:Battery3
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b'), 	# Goal:Battery4->Initial:Battery3
('0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67', '0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff'), 	# WP1:Battery4->WP2:Battery4
('0.3-destination-958d3ac7-fca3-43a1-b1d0-c7863d690fff', '0.3-destination-bba65014-29b1-42c8-82f6-938f73b02c67'), 	# WP2:Battery4->WP1:Battery4
('0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af', '0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05'), 	# Goal:Battery4->Initial:Battery4

#===================================================================================================================================
#   Plan 2
#===================================================================================================================================

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # Cable 1 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # Cable 1 Human->Initial
("0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f","0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682"), # Cable 1 Initial->WP1
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-destination-11b31593-1bf2-49fc-81c5-c63f8467b93b"), # Cable 1 WP1->Goal
("0.3-destination-11b31593-1bf2-49fc-81c5-c63f8467b93b","0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682"), # Cable 1 Goal->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # Cable 2 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # Cable 2 Human->Initial
("0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3","0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c"), # Cable 2 Initial->WP1
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-destination-7ab832de-bf80-40af-bbb1-ccc2f61745d7"), # Cable 2 WP1->Goal
("0.3-destination-7ab832de-bf80-40af-bbb1-ccc2f61745d7","0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c"), # Cable 2 Goal->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # Nut 1 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # Nut 1 Human->Initial
("0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20","0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc"), # Nut 1 Initial->WP1
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-destination-a47a0db9-d78c-4d1d-9136-69c2a655fac2"), # Nut 1 WP1->WP2
("0.3-destination-a47a0db9-d78c-4d1d-9136-69c2a655fac2","0.3-destination-acb60d50-d8b3-476e-973d-6a41f2b785ff"), # Nut 1 WP2->WP3
("0.3-destination-acb60d50-d8b3-476e-973d-6a41f2b785ff","0.3-destination-e41bcded-a73d-49ad-809e-6516462f937b"), # Nut 1 WP3->WP4
("0.3-destination-e41bcded-a73d-49ad-809e-6516462f937b","0.3-destination-ae1a2b11-a82b-4ce7-802e-c84d26b2fe42"), # Nut 1 WP4->Goal
("0.3-destination-ae1a2b11-a82b-4ce7-802e-c84d26b2fe42","0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc"), # Nut 1 Goal->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # Nut 2 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # Nut 2 Human->Initial
("0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c","0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675"), # Nut 2 Initial->WP1
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-destination-21fd6796-0822-49fe-a100-6e137692e4f8"), # Nut 2 WP1->WP2
("0.3-destination-21fd6796-0822-49fe-a100-6e137692e4f8","0.3-destination-be314c02-e493-4c66-9080-af4578fc997a"), # Nut 2 WP2->WP3
("0.3-destination-be314c02-e493-4c66-9080-af4578fc997a","0.3-destination-5e5f77b9-a47b-4da7-bdca-2575bc2fac58"), # Nut 2 WP3->WP4
("0.3-destination-5e5f77b9-a47b-4da7-bdca-2575bc2fac58","0.3-destination-5167443c-508e-4f47-8713-015d2aec83a5"), # Nut 2 WP4->Goal
("0.3-destination-5167443c-508e-4f47-8713-015d2aec83a5","0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675"), # Nut 2 Goal->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # Nut 3 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # Nut 3 Human->Initial
("0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901","0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b"), # Nut 3 Initial->WP1
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-destination-42c9a606-bdf2-4cb5-8ce3-74ec8d44ac40"), # Nut 3 WP1->WP2
("0.3-destination-42c9a606-bdf2-4cb5-8ce3-74ec8d44ac40","0.3-destination-72d939c6-c621-428b-bf9a-5603d72e9558"), # Nut 3 WP2->WP3
("0.3-destination-72d939c6-c621-428b-bf9a-5603d72e9558","0.3-destination-471568a0-bb68-47d9-89e7-207317e70200"), # Nut 3 WP3->WP4
("0.3-destination-471568a0-bb68-47d9-89e7-207317e70200","0.3-destination-dcd7b30c-0fe7-47ba-b976-6419784a4a5a"), # Nut 3 WP4->Goal
("0.3-destination-dcd7b30c-0fe7-47ba-b976-6419784a4a5a","0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b"), # Nut 3 Goal->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # Nut 4 Robot->Initial
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # Nut 4 Human->Initial
("0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af","0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e"), # Nut 4 Initial->WP1
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-destination-22a93ebe-4451-4295-ac09-a299a0c64b40"), # Nut 4 WP1->WP2
("0.3-destination-22a93ebe-4451-4295-ac09-a299a0c64b40","0.3-destination-053046e8-7953-42ff-ac1a-8c1adce3c253"), # Nut 4 WP2->WP3
("0.3-destination-053046e8-7953-42ff-ac1a-8c1adce3c253","0.3-destination-40f8d09b-addc-4a60-98f8-c329d5728979"), # Nut 4 WP3->WP4
("0.3-destination-40f8d09b-addc-4a60-98f8-c329d5728979","0.3-destination-cdfe7980-7ea7-4578-8e6f-beddcdfe313f"), # Nut 4 WP4->Goal
("0.3-destination-cdfe7980-7ea7-4578-8e6f-beddcdfe313f","0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e"), # Nut 4 Goal->WP1

("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682"), # Cable 1: Human->WP1
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c"), # Cable 2: Human->WP1
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc"), # Nut 1: Human->WP1
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675"), # Nut 2: Human->WP1
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b"), # Nut 3: Human->WP1
("0.3-agent-c3ca83fb-58e7-4c14-a677-6d078a3b9e09","0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e"), # Nut 4: Human->WP1

("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682"), # Cable 1: Robot->WP1
("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c"), # Cable 2: Robot->WP1
("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc"), # Nut 1: Robot->WP1
("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675"), # Nut 2: Robot->WP1
("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b"), # Nut 3: Robot->WP1
("0.3-agent-27180d4b-02f0-44b5-829a-e6045e1632a8","0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e"), # Nut 4: Robot->WP1

("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Cable1->Initial:Cable1
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Cable1->Initial:Cable2
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Cable1->Initial:Nut1
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Cable1->Initial:Nut2
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Cable1->Initial:Nut3
("0.3-destination-f67cec51-e946-4d56-ac8b-6094dee16682","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Cable1->Initial:Nut4

("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Cable2->Initial:Cable1
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Cable2->Initial:Cable2
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Cable2->Initial:Nut1
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Cable2->Initial:Nut2
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Cable2->Initial:Nut3
("0.3-destination-430f4790-7df6-4536-a86a-e32b045d441c","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Cable2->Initial:Nut4

("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Nut1->Initial:Cable1
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Nut1->Initial:Cable2
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Nut1->Initial:Nut1
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Nut1->Initial:Nut2
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Nut1->Initial:Nut3
("0.3-destination-da575ca5-0f81-409b-becf-0b0394b09fdc","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Nut1->Initial:Nut4

("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Nut2->Initial:Cable1
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Nut2->Initial:Cable2
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Nut2->Initial:Nut1
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Nut2->Initial:Nut2
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Nut2->Initial:Nut3
("0.3-destination-99bacc2a-46da-4001-b163-0596da3ee675","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Nut2->Initial:Nut4

("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Nut3->Initial:Cable1
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Nut3->Initial:Cable2
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Nut3->Initial:Nut1
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Nut3->Initial:Nut2
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Nut3->Initial:Nut3
("0.3-destination-e238a724-e161-4c8a-bae7-89a4e937e88b","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Nut3->Initial:Nut4

("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f"), # WP1:Nut4->Initial:Cable1
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3"), # WP1:Nut4->Initial:Cable2
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20"), # WP1:Nut4->Initial:Nut1
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c"), # WP1:Nut4->Initial:Nut2
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901"), # WP1:Nut4->Initial:Nut3
("0.3-destination-d4abb3f2-4b41-4daf-a65f-dbb832f50c5e","0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af"), # WP1:Nut4->Initial:Nut4

#===================================================================================================================================
#   Plan 3
#===================================================================================================================================

('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# Initial:Robot->WP1:Old Component1
('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# Initial:Robot->WP1:New Component1
('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# Initial:Robot->WP2:Old Component1
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP1:Old Component1->Initial:Robot
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP1:New Component1->Initial:Robot
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP2:Old Component1->Initial:Robot
('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# Initial:Robot->WP1:Old Component2
('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# Initial:Robot->WP1:New Component2
('0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# Initial:Robot->WP2:Old Component2
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP1:Old Component2->Initial:Robot
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP1:New Component2->Initial:Robot
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-agent-6bc862e4-1ebd-4e36-a948-4afe77a41dfa'), 	# WP2:Old Component2->Initial:Robot
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# Initial:Human->WP1:Old Component1
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# Initial:Human->WP1:New Component1
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# Initial:Human->WP2:Old Component1
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP1:Old Component1->Initial:Human
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP1:New Component1->Initial:Human
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP2:Old Component1->Initial:Human
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# Initial:Human->WP1:Old Component2
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# Initial:Human->WP1:New Component2
('0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# Initial:Human->WP2:Old Component2
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP1:Old Component2->Initial:Human
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP1:New Component2->Initial:Human
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-agent-3b5ff4a7-0882-45b1-bb46-9715b4049aeb'), 	# WP2:Old Component2->Initial:Human
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-thing-b2b860bb-f37a-4e77-a84e-e2fbc1153509'), 	# WP1:Old Component1->Initial:Old Component1
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-thing-22cdc687-ef41-4c35-9b0e-f2af8b75eec9'), 	# WP1:New Component1->Initial:New Component1
('0.3-thing-b2b860bb-f37a-4e77-a84e-e2fbc1153509', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# Initial:Old Component1->WP1:Old Component1
('0.3-thing-22cdc687-ef41-4c35-9b0e-f2af8b75eec9', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# Initial:New Component1->WP1:New Component1
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-destination-68fca638-95c3-4090-be0b-41828dc18122'), 	# WP2:Old Component1->Goal:Old Component1
('0.3-destination-68fca638-95c3-4090-be0b-41828dc18122', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# Goal:Old Component1->WP2:Old Component1
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-thing-169c5ff9-271f-4cd9-8e61-05cae9362815'), 	# WP1:Old Component2->Initial:Old Component2
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-thing-364a7ea3-36fb-4eea-a654-1eaff059c0f7'), 	# WP1:New Component2->Initial:New Component2
('0.3-thing-169c5ff9-271f-4cd9-8e61-05cae9362815', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# Initial:Old Component2->WP1:Old Component2
('0.3-thing-364a7ea3-36fb-4eea-a654-1eaff059c0f7', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# Initial:New Component2->WP1:New Component2
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-destination-7d70a1ed-4043-47be-8803-8948c0d3e3d4'), 	# WP2:Old Component2->Goal:Old Component2
('0.3-destination-7d70a1ed-4043-47be-8803-8948c0d3e3d4', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# Goal:Old Component2->WP2:Old Component2
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# WP1:Old Component1->WP2:Old Component1
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# WP1:New Component1->WP2:Old Component1
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# WP1:New Component1->WP1:Old Component1
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# WP1:Old Component1->WP1:New Component1
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# WP2:Old Component1->WP1:New Component1
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# WP2:Old Component1->WP1:Old Component1
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# WP1:Old Component1->WP2:Old Component2
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# WP1:New Component1->WP2:Old Component2
('0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# WP1:New Component1->WP1:Old Component2
('0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# WP1:Old Component1->WP1:New Component2
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# WP2:Old Component1->WP1:New Component2
('0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# WP2:Old Component1->WP1:Old Component2
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# WP1:Old Component2->WP2:Old Component1
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-destination-e3e6121a-bed6-475b-9b7d-8f440f2e7beb'), 	# WP1:New Component2->WP2:Old Component1
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# WP1:New Component2->WP1:Old Component1
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# WP1:Old Component2->WP1:New Component1
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-destination-1b652909-e2e1-4f24-af23-310cae45608a'), 	# WP2:Old Component2->WP1:New Component1
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-destination-89670c47-85cc-4b2c-8d8d-78eb48d850ce'), 	# WP2:Old Component2->WP1:Old Component1
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# WP1:Old Component2->WP2:Old Component2
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8'), 	# WP1:New Component2->WP2:Old Component2
('0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# WP1:New Component2->WP1:Old Component2
('0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# WP1:Old Component2->WP1:New Component2
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-destination-77ad5753-fcb9-4837-a3db-ce64c68d9549'), 	# WP2:Old Component2->WP1:New Component2
('0.3-destination-6962ea85-fbb4-491b-9555-06d71e9ecaa8', '0.3-destination-c439acc3-9416-4388-8d41-a3aecc9e832a'), 	# WP2:Old Component2->WP1:Old Component2


#===================================================================================================================================
#   FIN
#===================================================================================================================================

]

release_wl = [

 # Plan 1
 "0.3-destination-887dff63-0217-4d90-b3a6-d1c2cab6c0af",
 "0.3-destination-7a9c0d69-04c9-48ad-a78f-54afac8ef224",
 "0.3-destination-01f1c1d8-b933-4962-aff5-296bed236111",
 "0.3-thing-af10da1f-c8a2-4282-b33f-5be927c2e549",
 "0.3-thing-8994af3e-13ec-44cf-b675-b1926ae43d05",
 "0.3-thing-7bf1d97e-607f-4d7f-9fbd-ac2d95f718a8",
 "0.3-thing-029c036a-3a17-4723-bb30-775ef7fb4e0b",
 "0.3-thing-4c01555d-755e-493c-ad0f-3614bf6c6de4",
 "0.3-destination-396af1f7-bbdb-4a14-a29c-6df1bb0358af",
 "0.3-thing-4d002108-9699-49da-a5f2-57720e4d431a",
 "0.3-destination-ffb7b690-c05f-41cd-bc23-308b2866983c",
 "0.3-destination-81fdec32-175d-49c9-b1df-2e3eef73ac6e",
 "0.3-thing-ee20cbce-84cc-4445-94b8-e541bb5ae5c5",
 "0.3-thing-d879237d-900f-4740-9786-ed27e38dca06",
 "0.3-destination-5f5b234e-a04e-44a4-bc35-3c983771b01f",
 "0.3-thing-02c8aa84-5343-48a1-beb1-d6a0d0f2b459",
 "0.3-destination-d7f2046c-12fa-4c08-a707-b143d7b2e654",
 "0.3-thing-6b88713a-a8d6-407c-a4f1-f2575b78ef57",
 "0.3-thing-16bf178c-75bf-4a28-9ee2-710a88cfcb94",
 "0.3-thing-4c3ccadc-a0f0-4b8e-a733-9f003dbf4e22",

 # Plan 2
 "0.3-thing-b0220036-02ea-4f7a-ab95-314cdbd9d27f",
 "0.3-destination-11b31593-1bf2-49fc-81c5-c63f8467b93b",
 "0.3-destination-7ab832de-bf80-40af-bbb1-ccc2f61745d7",
 "0.3-destination-ae1a2b11-a82b-4ce7-802e-c84d26b2fe42",
 "0.3-destination-5167443c-508e-4f47-8713-015d2aec83a5",
 "0.3-destination-dcd7b30c-0fe7-47ba-b976-6419784a4a5a",
 "0.3-destination-cdfe7980-7ea7-4578-8e6f-beddcdfe313f",
 "0.3-thing-a506b5ea-87cf-4b62-a777-d4bd9707f9b3",
 "0.3-thing-bade4dfc-6db4-4fb8-857f-a79541128e20",
 "0.3-thing-cbf628b3-32c9-4e0d-916c-5bff3ac00b3c",
 "0.3-thing-d24e74ef-d343-4419-8426-dc8a16fdc901",
 "0.3-thing-25b37bf1-6904-45a6-8396-30f8291b59af",

 # Plan 3
 "0.3-destination-68fca638-95c3-4090-be0b-41828dc18122",
 "0.3-destination-7d70a1ed-4043-47be-8803-8948c0d3e3d4",
 "0.3-thing-22cdc687-ef41-4c35-9b0e-f2af8b75eec9",
 "0.3-thing-364a7ea3-36fb-4eea-a654-1eaff059c0f7",
 "0.3-thing-b2b860bb-f37a-4e77-a84e-e2fbc1153509",
 "0.3-thing-169c5ff9-271f-4cd9-8e61-05cae9362815"
]

def in_transport_wl(from_dest,to_dest):
    return (from_dest,to_dest) in transport_wl


def in_release_wl(dest):
    return dest in release_wl
