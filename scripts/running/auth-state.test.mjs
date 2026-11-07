import test from 'node:test'
import assert from 'node:assert/strict'
import { randomBytes } from 'node:crypto'
import { encryptState, decryptState, parseKey } from './auth-state.mjs'

const state = { issuer: 'https://mcpcn.coros.com', token: { access_token: 'test-access', refresh_token: 'test-refresh', expires_at_epoch: 1799999999, client_id: 'test-client' } }

test('授权加密可还原，密文不包含原始 token', () => {
  const key = randomBytes(32)
  const encrypted = encryptState(state, key)
  assert.deepEqual(decryptState(encrypted, key), state)
  assert.ok(!JSON.stringify(encrypted).includes('test-access'))
  assert.notEqual(encryptState(state, key).iv, encrypted.iv)
})

test('错误密钥、被篡改的密文和未知地区必须被拒绝', () => {
  const key = randomBytes(32)
  const encrypted = encryptState(state, key)
  assert.throws(() => decryptState(encrypted, randomBytes(32)))
  const raw = Buffer.from(encrypted.ciphertext, 'base64')
  raw[0] ^= 1
  assert.throws(() => decryptState({ ...encrypted, ciphertext: raw.toString('base64') }, key))
  assert.throws(() => encryptState({ ...state, issuer: 'https://example.com' }, key))
  assert.throws(() => parseKey('invalid'))
  assert.deepEqual(parseKey(key.toString('base64')), key)
})
