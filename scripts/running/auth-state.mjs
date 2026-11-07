import { createCipheriv, createDecipheriv, randomBytes } from 'node:crypto'
import { readFile, writeFile, mkdir, rename } from 'node:fs/promises'
import { fileURLToPath } from 'node:url'
import { dirname, resolve } from 'node:path'

const root = fileURLToPath(new URL('../../', import.meta.url))
const keyPath = resolve(root, '.local/coros/github-state-key.txt')
const encryptedPath = resolve(root, '.github/coros-auth.enc')
const purpose = Buffer.from('yyx-blog/coros-auth/v1')
const regions = { 'https://mcpcn.coros.com': 'cn', 'https://mcpeu.coros.com': 'eu', 'https://mcpus.coros.com': 'us', 'https://mcp.coros.com': 'mcp-coros-com' }

export function parseKey(value) {
  const key = Buffer.from(value.trim(), 'base64')
  if (key.length !== 32 || key.toString('base64') !== value.trim()) throw new Error('COROS_STATE_KEY 必须是 32 字节密钥的 Base64 编码。')
  return key
}

export function validateState(state) {
  if (!regions[state?.issuer] || typeof state?.token?.access_token !== 'string' || !state.token.access_token
    || typeof state.token.refresh_token !== 'string' || !state.token.refresh_token
    || typeof state.token.client_id !== 'string' || !Number.isFinite(state.token.expires_at_epoch)) throw new Error('COROS 授权状态无效，请重新登录。')
  return state
}

export function encryptState(state, key) {
  validateState(state)
  const iv = randomBytes(12)
  const cipher = createCipheriv('aes-256-gcm', key, iv)
  cipher.setAAD(purpose)
  const ciphertext = Buffer.concat([cipher.update(JSON.stringify(state)), cipher.final()])
  return { version: 1, iv: iv.toString('base64'), tag: cipher.getAuthTag().toString('base64'), ciphertext: ciphertext.toString('base64') }
}

export function decryptState(encrypted, key) {
  if (encrypted?.version !== 1) throw new Error('不支持的授权状态版本。')
  const decipher = createDecipheriv('aes-256-gcm', key, Buffer.from(encrypted.iv, 'base64'))
  decipher.setAAD(purpose)
  decipher.setAuthTag(Buffer.from(encrypted.tag, 'base64'))
  const plaintext = Buffer.concat([decipher.update(Buffer.from(encrypted.ciphertext, 'base64')), decipher.final()])
  return validateState(JSON.parse(plaintext.toString('utf8')))
}

async function loadKey() {
  const value = process.env.COROS_STATE_KEY || await readFile(keyPath, 'utf8')
  return parseKey(value)
}

async function main() {
  const command = process.argv[2]
  if (command === 'init') {
    await mkdir(dirname(keyPath), { recursive: true, mode: 0o700 })
    try { await writeFile(keyPath, `${randomBytes(32).toString('base64')}\n`, { mode: 0o600, flag: 'wx' }) }
    catch (error) { if (error.code !== 'EEXIST') throw error }
  } else if (!['pack', 'unpack'].includes(command)) throw new Error('用法：node scripts/running/auth-state.mjs init|pack|unpack')
  const key = await loadKey()
  if (command === 'unpack') {
    const state = decryptState(JSON.parse(await readFile(encryptedPath, 'utf8')), key)
    const tokenPath = resolve(root, '.local/coros', regions[state.issuer], 'token.json')
    await mkdir(dirname(tokenPath), { recursive: true, mode: 0o700 })
    await writeFile(tokenPath, JSON.stringify(state.token), { mode: 0o600 })
    console.log('已恢复 COROS 授权。')
    return
  }
  const issuer = process.env.COROS_ISSUER || 'https://mcpcn.coros.com'
  if (!regions[issuer]) throw new Error('未知 COROS 地区。')
  const token = JSON.parse(await readFile(resolve(root, '.local/coros', regions[issuer], 'token.json'), 'utf8'))
  const state = validateState({ issuer, token })
  let previous
  try { previous = decryptState(JSON.parse(await readFile(encryptedPath, 'utf8')), key) }
  catch (error) { if (error.code !== 'ENOENT') throw error }
  // Preserve ciphertext when the token did not rotate, avoiding daily Git churn.
  if (JSON.stringify(previous) !== JSON.stringify(state)) {
    await mkdir(dirname(encryptedPath), { recursive: true })
    await writeFile(`${encryptedPath}.tmp`, `${JSON.stringify(encryptState(state, key))}\n`, { mode: 0o600 })
    await rename(`${encryptedPath}.tmp`, encryptedPath)
  }
  console.log('授权已加密保存；未输出任何 token 或密钥。')
  if (command === 'init') console.log('将 .local/coros/github-state-key.txt 的内容设置为 GitHub Actions Secret：COROS_STATE_KEY。')
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  main().catch(() => { console.error('授权状态处理失败，请检查密钥、地区和本机登录状态。'); process.exitCode = 1 })
}
