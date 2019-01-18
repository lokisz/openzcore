//
// reg test template
//
	// write register
	addis r4,0,REG_ADDR@h
	ori r4,r4,REG_ADDR@l
	addis r5,0,REG_DATA@h
	ori r5,r5,REG_DATA@l
	stw r5,0(r4)

	// read register
	lwz r6,0(r4)

	// Compare write/read value
	cmp 0,0,r5,r6
    bne err_handler

    b done

//
// Read all regs
//
reg_address_space:
	addis r4,0,SIUMCR_ADDR@h
	ori r4,r4,SIUMCR_ADDR@l
	addis r10,0,MCCF2_ADDR@h
	ori r10,r10,MCCF2_ADDR@l

loop1:
	lwz r5,0(r4)
	addi r4,r4,4

	cmp 0,0,r4,r10
	bne loop1

//
// Write all regs
//
    addis r4,0,SIUMCR_ADDR@h
    ori r4,r4,SIUMCR_ADDR@l
	addis r10,0,MCCF2_ADDR@h
	ori r10,r10,MCCF2_ADDR@l

loop2:
	lwz r5,0(r4)
    stw r5,0(r4)

    addi r4,r4,4
	cmp 0,0,r4,r10
	bne loop2

//
// Memory Initialization
//
memory_space:
	addis r4,0,MEM_START@h
	ori r4,r4,MEM_START@l
	addis r10,0,MEM_END@h
	ori r10,r10,MEM_END@l

loop:
	stw r0,0(r4)
	addi r4,r4,4

	cmp 0,0,r4,r10
	bne loop

//
// Memory Test:
//      Write address value to memory
//
memory_space:
	addis r4,0,MEM_START@h
	ori r4,r4,MEM_START@l
	addis r10,0,MEM_END@h
	ori r10,r10,MEM_END@l

loop:
	stw r4,0(r4)
	lwz r9,0(r4)
	cmp 0,0,r4,r9
	bne err_handler

    addis r5,0,0x0000
    ori r5,r5,0x0000
	stw r5,0(r4)
	lwz r9,0(r4)
	cmp 0,0,r5,r9
	bne err_handler

    addis r6,0,0xffff
    ori r6,r6,0xffff
	stw r6,0(r4)
	lwz r9,0(r4)
	cmp 0,0,r6,r9
	bne err_handler

	addi r4,r4,4
	cmp 0,0,r4,r10
	bne loop

//
//  bcl---touch vector: Just note program has touched it
//
    addis r30,0,FLAG_NUM@h       // You should replace FLAG_NUM with interger to flag touch
    ori r30,r30,FLAG_NUM@l
    bnel touch_vector
    ...

//
// Lock primitive for parallel programming
//
lab1:
    lwarx r7,r0,r31
    cmpwi r7,0x0
    bne lab1     // if r7!=0, goto label_A, else continue
    
    stwcx. r1,r0,r31
    bne lab1     // if store is successful, enter critical section, else goto label_A

Critical sections:
    ...             

Lock release:
    li r1,0
    stw r1,0(r31)   // load 0 t r1 and store to r31

